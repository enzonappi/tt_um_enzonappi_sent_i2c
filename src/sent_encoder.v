`default_nettype none
`timescale 1ns / 1ps

// SENT (SAE J2716) single-channel transmitter/encoder.
//
// Generates a real SENT waveform (a train of falling edges spaced by
// tick-accurate periods: 56 ticks sync, then 12+value ticks per status/data
// nibble, then 12+value ticks for the CRC nibble, then -- in sync mode -- a
// pause pulse that pads the total frame to a fixed tick count). Unlike the
// decoder, the tick length here is not measured: it is supplied directly
// (tick_cycles, in raw clock cycles) since there is nothing to calibrate
// against on the transmit side.
//
// The data nibble count (num_nibbles_cfg) is latched once per frame, at the
// moment the sync pulse starts, so it can't change the loop bound mid-frame;
// the actual status/data nibble *values* are read live at each nibble
// boundary, so a write to the TX payload registers is picked up as soon as
// possible instead of waiting for the next frame -- if that matters for your
// use, write the payload while tx_enable=0 or while tx_busy=0.
module sent_encoder #(
    parameter integer DATA_NIBBLES = 6,   // 1-6 per SAE J2716
    parameter integer TICK_WIDTH   = 9,   // width of ticks_left / elapsed_ticks / frame_duration_ticks
    parameter integer CYCLE_WIDTH  = 8    // width of tick_cycles (clk cycles per tick), independent of TICK_WIDTH
) (
    input  wire clk,
    input  wire rst_n,

    // runtime configuration (from the I2C TX_CONFIG/TX_TICK/TX_FRAME_DUR registers)
    input  wire                      tx_enable,             // 1 = transmit frames back-to-back
    input  wire                      invert_tx,             // 1 = invert sent_out
    input  wire                      sync_mode,             // 1 = pad each frame to frame_duration_ticks with a pause
    input  wire [2:0]                num_nibbles_cfg,       // data nibbles per frame, 1-6 (0 or >6 clamps to 6)
    input  wire [3:0]                status_nibble_tx,      // TX payload (from I2C write registers)
    input  wire [4*DATA_NIBBLES-1:0] data_nibbles_tx,       // nibble i in bits [4*i +: 4]
    input  wire [CYCLE_WIDTH-1:0]    tick_cycles,           // clk cycles per tick (0 treated as 1)
    input  wire [TICK_WIDTH-1:0]     frame_duration_ticks,  // sync-mode target total frame length, in ticks

    output wire sent_out,     // the transmitted SENT waveform
    output wire tx_busy       // 0 while idle (tx_enable=0 and no frame in flight)
);

    localparam integer TICKS_SYNC       = 56;
    localparam integer NIBBLE_MIN_TICKS = 12;
    localparam [3:0]   CRC_INIT         = 4'h5;

    localparam [2:0] TXST_IDLE   = 3'd0,
                     TXST_SYNC   = 3'd1,
                     TXST_STATUS = 3'd2,
                     TXST_DATA   = 3'd3,
                     TXST_CRC    = 3'd4,
                     TXST_PAUSE  = 3'd5;

    wire [2:0] num_nibbles_eff_live =
        (num_nibbles_cfg == 3'd0 || num_nibbles_cfg > DATA_NIBBLES[2:0]) ?
        DATA_NIBBLES[2:0] : num_nibbles_cfg;

    wire [CYCLE_WIDTH-1:0] tick_cycles_eff =
        (tick_cycles == {CYCLE_WIDTH{1'b0}}) ? {{(CYCLE_WIDTH-1){1'b0}}, 1'b1} : tick_cycles;

    function [3:0] crc4_step;
        input [3:0] crc_in;
        input [3:0] nibble;
        reg   [3:0] c;
        integer i;
        begin
            c = crc_in ^ nibble;
            for (i = 0; i < 4; i = i + 1)
                c = c[3] ? ({c[2:0], 1'b0} ^ 4'h3) : {c[2:0], 1'b0};
            crc4_step = c;
        end
    endfunction

    // ---------------------------------------------------------------
    // Tick generator: 1-cycle pulse every tick_cycles_eff clk cycles
    // ---------------------------------------------------------------
    reg [CYCLE_WIDTH-1:0] tick_cyc_cnt;
    wire tick_pulse = (tick_cyc_cnt == tick_cycles_eff - 1'b1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          tick_cyc_cnt <= {CYCLE_WIDTH{1'b0}};
        else if (tick_pulse) tick_cyc_cnt <= {CYCLE_WIDTH{1'b0}};
        else                 tick_cyc_cnt <= tick_cyc_cnt + 1'b1;
    end

    // ---------------------------------------------------------------
    // Main FSM
    // ---------------------------------------------------------------
    reg [2:0]               phase;
    reg                      out_r;
    reg [TICK_WIDTH-1:0]     ticks_left;    // ticks remaining in the current period
    reg [2:0]                data_idx;      // 1..nnib_shadow while phase==TXST_DATA
    reg [TICK_WIDTH-1:0]     elapsed_ticks; // ticks used so far this frame, up to (not incl.) CRC
    reg [3:0]                crc_accum;

    // only the nibble *count* is latched per frame (it sets the data loop's
    // bound); the nibble *values* are read live -- see the module header
    reg [2:0] nnib_shadow;

    assign sent_out = out_r ^ invert_tx;
    assign tx_busy  = (phase != TXST_IDLE);

    wire [3:0] cur_data_nibble  = data_nibbles_tx[(data_idx-1)*4 +: 4];
    wire [3:0] next_data_nibble = data_nibbles_tx[data_idx*4 +: 4];
    wire [3:0] crc_fold_val     = (phase == TXST_STATUS) ? status_nibble_tx : cur_data_nibble;
    wire [3:0] crc_next         = crc4_step(crc_accum, crc_fold_val);

    wire [TICK_WIDTH-1:0] ticks_before_pause = elapsed_ticks + NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + crc_accum;
    wire pause_underrun = (frame_duration_ticks <= ticks_before_pause);
    wire [TICK_WIDTH-1:0] pause_ticks =
        pause_underrun ? {{(TICK_WIDTH-1){1'b0}}, 1'b1} : (frame_duration_ticks - ticks_before_pause);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase         <= TXST_IDLE;
            out_r         <= 1'b1;
            ticks_left    <= {TICK_WIDTH{1'b0}};
            data_idx      <= 3'd0;
            elapsed_ticks <= {TICK_WIDTH{1'b0}};
            crc_accum     <= CRC_INIT;
            nnib_shadow   <= 3'd6;
        end else begin
            if (tick_pulse) begin
                if (ticks_left != {TICK_WIDTH{1'b0}}) begin
                    // still inside the current period
                    ticks_left <= ticks_left - 1'b1;
                    out_r      <= 1'b1;
                end else begin
                    // current period just ended: move on
                    case (phase)

                        TXST_IDLE: begin
                            if (tx_enable) begin
                                nnib_shadow   <= num_nibbles_eff_live;
                                crc_accum     <= CRC_INIT;
                                elapsed_ticks <= TICKS_SYNC[TICK_WIDTH-1:0];
                                out_r         <= 1'b0;
                                ticks_left    <= TICKS_SYNC[TICK_WIDTH-1:0] - 1'b1;
                                phase         <= TXST_SYNC;
                            end
                        end

                        TXST_SYNC: begin
                            out_r      <= 1'b0;
                            ticks_left <= (NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + status_nibble_tx) - 1'b1;
                            phase      <= TXST_STATUS;
                        end

                        TXST_STATUS: begin
                            crc_accum     <= crc_next;
                            elapsed_ticks <= elapsed_ticks + NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + status_nibble_tx;
                            data_idx      <= 3'd1;
                            out_r         <= 1'b0;
                            ticks_left    <= (NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + data_nibbles_tx[3:0]) - 1'b1;
                            phase         <= TXST_DATA;
                        end

                        TXST_DATA: begin
                            crc_accum     <= crc_next;
                            elapsed_ticks <= elapsed_ticks + NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + cur_data_nibble;
                            out_r         <= 1'b0;
                            if (data_idx == nnib_shadow) begin
                                ticks_left <= (NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + crc_next) - 1'b1;
                                phase      <= TXST_CRC;
                            end else begin
                                data_idx   <= data_idx + 1'b1;
                                ticks_left <= (NIBBLE_MIN_TICKS[TICK_WIDTH-1:0] + next_data_nibble) - 1'b1;
                                phase      <= TXST_DATA;
                            end
                        end

                        TXST_CRC: begin
                            if (sync_mode) begin
                                out_r       <= 1'b0;
                                ticks_left  <= pause_ticks - 1'b1;
                                phase       <= TXST_PAUSE;
                            end else if (tx_enable) begin
                                nnib_shadow   <= num_nibbles_eff_live;
                                crc_accum     <= CRC_INIT;
                                elapsed_ticks <= TICKS_SYNC[TICK_WIDTH-1:0];
                                out_r         <= 1'b0;
                                ticks_left    <= TICKS_SYNC[TICK_WIDTH-1:0] - 1'b1;
                                phase         <= TXST_SYNC;
                            end else begin
                                out_r      <= 1'b1;
                                ticks_left <= {TICK_WIDTH{1'b0}};
                                phase      <= TXST_IDLE;
                            end
                        end

                        TXST_PAUSE: begin
                            if (tx_enable) begin
                                nnib_shadow   <= num_nibbles_eff_live;
                                crc_accum     <= CRC_INIT;
                                elapsed_ticks <= TICKS_SYNC[TICK_WIDTH-1:0];
                                out_r         <= 1'b0;
                                ticks_left    <= TICKS_SYNC[TICK_WIDTH-1:0] - 1'b1;
                                phase         <= TXST_SYNC;
                            end else begin
                                out_r      <= 1'b1;
                                ticks_left <= {TICK_WIDTH{1'b0}};
                                phase      <= TXST_IDLE;
                            end
                        end

                        default: phase <= TXST_IDLE;

                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
