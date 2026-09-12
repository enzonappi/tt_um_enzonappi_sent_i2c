`default_nettype none
`timescale 1ns / 1ps

// SENT (SAE J2716) single-channel decoder.
//
// The tick length is not assumed fixed: it is measured on every frame from
// the calibration/sync pulse (nominal TICKS_SYNC=56 tick units) and used to
// decode the following nibbles. sync_min_cycles/sync_max_cycles bound the
// *raw clock-cycle* length of a plausible sync pulse, so the same RTL can
// lock onto sensors that use a different tick unit -- these are runtime
// register inputs (see the I2C SYNC_MIN_L/H, SYNC_MAX_L/H registers in
// tt_um_enzonappi_sent_i2c.v), computed as:
//   sync_min_cycles = TICKS_SYNC * (tick_time_min_s * CLK_FREQ_HZ)
//   sync_max_cycles = TICKS_SYNC * (tick_time_max_s * CLK_FREQ_HZ)
// The reset default (set upstream, before any I2C write) assumes a 10 MHz
// system clock and accepts any tick length between 1 us and 6 us (nominal
// automotive SENT tick is 3 us).
module sent_decoder #(
    parameter integer DATA_NIBBLES     = 6,     // 1-6 per SAE J2716
    parameter integer DIV_WIDTH        = 16     // width of the shared divider / period counters
) (
    input  wire clk,
    input  wire rst_n,
    input  wire sent_in,

    // runtime configuration (from the I2C CONFIG register)
    input  wire        invert_sent,      // 1 = invert sent_in before decoding
    input  wire        use_pause,        // 1 = an extra pause pulse follows the CRC nibble
    input  wire  [2:0] num_nibbles_cfg,  // data nibbles per frame, 1-6 (0 or >6 clamps to 6)

    // runtime configuration (from the I2C SYNC_MIN/MAX registers)
    input  wire [DIV_WIDTH-1:0] sync_min_cycles,
    input  wire [DIV_WIDTH-1:0] sync_max_cycles,

    output reg                        frame_valid,  // 1-cycle pulse: new frame latched
    output reg  [3:0]                 status_nibble,
    output reg  [4*DATA_NIBBLES-1:0]  data_nibbles,  // nibble i in bits [4*i +: 4]
    output reg  [3:0]                 crc_nibble,
    output reg                        crc_error,     // 1-cycle pulse, coincides with frame_valid
    output reg                        sync_error,    // 1-cycle pulse: lost sync / bad nibble / timeout
    output wire                       frame_busy     // 0 while searching for sync, 1 while capturing a frame
);

    localparam integer TICKS_SYNC       = 56;
    localparam integer NIBBLE_MIN_TICKS = 12;   // nibble value 0 encodes as 12 ticks, value 15 as 27 ticks
    localparam [3:0]   CRC_INIT         = 4'h5; // SAE J2716 recommended seed

    // num_nibbles_cfg=0 or >DATA_NIBBLES clamps to DATA_NIBBLES (the compiled-in maximum,
    // which also sizes the data_nibbles bus and can't itself change at runtime)
    wire [2:0] num_nibbles_eff =
        (num_nibbles_cfg == 3'd0 || num_nibbles_cfg > DATA_NIBBLES[2:0]) ?
        DATA_NIBBLES[2:0] : num_nibbles_cfg;
    wire [3:0] num_nibbles_total = {1'b0, num_nibbles_eff} + 4'd2; // status + data + crc

    localparam [2:0] ST_SEARCH    = 3'd0,
                     ST_CAL       = 3'd1,
                     ST_WAIT_E    = 3'd2,
                     ST_DIV_N     = 3'd3,
                     ST_EVAL_N    = 3'd4,
                     ST_LATCH     = 3'd5,
                     ST_SKIP_PAUSE = 3'd6;

    // ---------------------------------------------------------------
    // Input synchronizer + falling-edge detector
    // ---------------------------------------------------------------
    wire      sent_in_eff = sent_in ^ invert_sent;
    reg [1:0] sent_sync;
    reg       sent_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sent_sync <= 2'b11;
            sent_prev <= 1'b1;
        end else begin
            sent_sync <= {sent_sync[0], sent_in_eff};
            sent_prev <= sent_sync[1];
        end
    end
    wire edge_fall = sent_prev & ~sent_sync[1];

    // ---------------------------------------------------------------
    // Free-running period counter: cycles since the previous falling edge
    // ---------------------------------------------------------------
    localparam [DIV_WIDTH-1:0] PERIOD_MAX = {DIV_WIDTH{1'b1}};
    reg [DIV_WIDTH-1:0] period_counter;
    reg [DIV_WIDTH-1:0] period;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_counter <= {DIV_WIDTH{1'b0}};
            period         <= {DIV_WIDTH{1'b0}};
        end else if (edge_fall) begin
            period         <= period_counter + 1'b1; // period_counter is 0 on the 1st cycle after
                                                       // reset, so it under-counts elapsed cycles by 1
            period_counter <= {DIV_WIDTH{1'b0}};
        end else if (period_counter != PERIOD_MAX) begin
            period_counter <= period_counter + 1'b1;
        end
    end

    // ---------------------------------------------------------------
    // Shared divider
    // ---------------------------------------------------------------
    reg                  div_start;
    reg  [DIV_WIDTH-1:0] div_dividend;
    reg  [DIV_WIDTH-1:0] div_divisor;
    wire [DIV_WIDTH-1:0] div_quotient;
    wire [DIV_WIDTH-1:0] div_remainder;
    wire                 div_busy;
    wire                 div_done;

    divider #(.WIDTH(DIV_WIDTH)) u_divider (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (div_start),
        .dividend  (div_dividend),
        .divisor   (div_divisor),
        .quotient  (div_quotient),
        .remainder (div_remainder),
        .busy      (div_busy),
        .done      (div_done)
    );

    // ---------------------------------------------------------------
    // CRC-4 (SAE J2716), one nibble per call: x^4 + x^3 + 1 (poly 0x13)
    // ---------------------------------------------------------------
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
    // Main FSM
    // ---------------------------------------------------------------
    reg [2:0]                state;
    reg [3:0]                nibble_idx;
    reg [DIV_WIDTH-1:0]      tick_period;
    reg [3:0]                status_nibble_r;
    reg [4*DATA_NIBBLES-1:0] data_nibbles_r;
    reg [3:0]                crc_nibble_r;
    reg [3:0]                crc_accum;

    assign frame_busy = (state != ST_SEARCH);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_SEARCH;
            nibble_idx      <= 4'd0;
            tick_period     <= {DIV_WIDTH{1'b0}};
            status_nibble_r <= 4'd0;
            data_nibbles_r  <= {(4*DATA_NIBBLES){1'b0}};
            crc_nibble_r    <= 4'd0;
            crc_accum       <= 4'd0;
            div_start       <= 1'b0;
            div_dividend    <= {DIV_WIDTH{1'b0}};
            div_divisor     <= {DIV_WIDTH{1'b0}};
            frame_valid     <= 1'b0;
            status_nibble   <= 4'd0;
            data_nibbles    <= {(4*DATA_NIBBLES){1'b0}};
            crc_nibble      <= 4'd0;
            crc_error       <= 1'b0;
            sync_error      <= 1'b0;
        end else begin
            div_start   <= 1'b0;
            frame_valid <= 1'b0;
            crc_error   <= 1'b0;
            sync_error  <= 1'b0;

            case (state)

                ST_SEARCH: begin
                    if (edge_fall &&
                        (period_counter + 1'b1) >= sync_min_cycles &&
                        (period_counter + 1'b1) <= sync_max_cycles) begin
                        div_dividend <= period_counter + 1'b1;
                        div_divisor  <= TICKS_SYNC[DIV_WIDTH-1:0];
                        div_start    <= 1'b1;
                        state        <= ST_CAL;
                    end
                end

                ST_CAL: begin
                    if (div_done) begin
                        tick_period    <= div_quotient;
                        nibble_idx     <= 4'd0;
                        crc_accum      <= CRC_INIT;
                        // clear stale upper nibbles in case num_nibbles_cfg was just lowered
                        data_nibbles_r <= {(4*DATA_NIBBLES){1'b0}};
                        state          <= ST_WAIT_E;
                    end
                end

                ST_WAIT_E: begin
                    if (edge_fall) begin
                        // round to the nearest tick by adding half a tick before dividing
                        // (+1 corrects period_counter's 0-indexing, see the period_counter block above)
                        div_dividend <= period_counter + 1'b1 + {1'b0, tick_period[DIV_WIDTH-1:1]};
                        div_divisor  <= tick_period;
                        div_start    <= 1'b1;
                        state        <= ST_DIV_N;
                    end else if (period_counter == PERIOD_MAX) begin
                        sync_error <= 1'b1;
                        state      <= ST_SEARCH;
                    end
                end

                ST_DIV_N: begin
                    if (div_done) begin
                        if (div_quotient < NIBBLE_MIN_TICKS[DIV_WIDTH-1:0] ||
                            div_quotient > (NIBBLE_MIN_TICKS[DIV_WIDTH-1:0] + 16'd15)) begin
                            sync_error <= 1'b1;
                            state      <= ST_SEARCH;
                        end else begin
                            state <= ST_EVAL_N;
                        end
                    end
                end

                ST_EVAL_N: begin
                    // div_quotient is still valid: holds the result latched by ST_DIV_N
                    if (nibble_idx == 4'd0) begin
                        status_nibble_r <= div_quotient[3:0] - NIBBLE_MIN_TICKS[3:0];
                        crc_accum       <= crc4_step(crc_accum, div_quotient[3:0] - NIBBLE_MIN_TICKS[3:0]);
                    end else if (nibble_idx <= {1'b0, num_nibbles_eff}) begin
                        data_nibbles_r[(nibble_idx-1)*4 +: 4] <= div_quotient[3:0] - NIBBLE_MIN_TICKS[3:0];
                        crc_accum <= crc4_step(crc_accum, div_quotient[3:0] - NIBBLE_MIN_TICKS[3:0]);
                    end else begin
                        crc_nibble_r <= div_quotient[3:0] - NIBBLE_MIN_TICKS[3:0];
                    end

                    if (nibble_idx == num_nibbles_total - 4'd1) begin
                        state <= ST_LATCH;
                    end else begin
                        nibble_idx <= nibble_idx + 1'b1;
                        state      <= ST_WAIT_E;
                    end
                end

                ST_LATCH: begin
                    status_nibble <= status_nibble_r;
                    data_nibbles  <= data_nibbles_r;
                    crc_nibble    <= crc_nibble_r;
                    crc_error     <= (crc_accum != crc_nibble_r);
                    frame_valid   <= 1'b1;
                    // a well-formed frame may append a pause pulse after the CRC nibble
                    // purely to pad the total frame time; consume that extra edge here
                    // so ST_SEARCH never mistakes it for the next sync candidate.
                    state         <= use_pause ? ST_SKIP_PAUSE : ST_SEARCH;
                end

                ST_SKIP_PAUSE: begin
                    if (edge_fall) begin
                        state <= ST_SEARCH;
                    end else if (period_counter == PERIOD_MAX) begin
                        sync_error <= 1'b1;
                        state      <= ST_SEARCH;
                    end
                end

                default: state <= ST_SEARCH;

            endcase
        end
    end

endmodule

`default_nettype wire
