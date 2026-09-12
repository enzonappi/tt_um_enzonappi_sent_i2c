`default_nettype none
`timescale 1ns / 1ps

// Tiny Tapeout top level: SENT (SAE J2716) decoder + encoder, full duplex,
// feeding/fed by a single I2C register-map slave.
//
// Pinout
//   ui_in[0]   : SENT signal in (receive)
//   ui_in[7:1] : unused
//   uio[0]     : I2C SCL  (input only, no clock stretching)
//   uio[1]     : I2C SDA  (open-drain: driven low or released)
//   uio[7:2]   : unused
//   uo_out[0]  : new_data  (live view of STATUS bit0)
//   uo_out[1]  : crc_error (live view of STATUS bit1)
//   uo_out[2]  : sync_error(live view of STATUS bit2)
//   uo_out[3]  : frame_busy
//   uo_out[4]  : SENT signal out (transmit) -- independent of ui_in[0], runs
//                concurrently with the receiver (true full duplex)
//   uo_out[5]  : tx_busy
//   uo_out[7:6]: 0
//
// I2C slave address: 0x50. Register map (8-bit regs, pointer auto-increments
// and wraps at NUM_REGS, set by writing 1 byte; a further optional data byte
// writes that register, for writable registers):
//
//   -- receive (unchanged) --
//   0x00 STATUS         {4'b0, frame_busy, sync_error, crc_error, new_data}
//                        (bits 0-2 are sticky and clear on reading this reg)
//   0x01 STATUS_NIBBLE   {4'b0, sent status/communication nibble}
//   0x02 DATA0           {nibble2, nibble1}
//   0x03 DATA1           {nibble4, nibble3}
//   0x04 DATA2           {nibble6, nibble5}
//   0x05 CRC             {4'b0, sent crc nibble}
//   0x06 FRAME_COUNT     free-running count of decoded frames (any CRC)
//   0x07 CONFIG          {3'b0, num_nibbles[2:0], use_pause, invert_sent}  (R/W)
//                        num_nibbles: 1-6 data nibbles per frame (0 or >6 -> 6)
//                        use_pause:   1 = an extra pause pulse follows the CRC nibble
//                        invert_sent: 1 = invert ui_in[0] before decoding
//
//   -- transmit (new) --
//   0x08 TX_STATUS_NIBBLE {4'b0, status nibble to transmit}                (R/W)
//   0x09 TX_DATA0         {nibble2, nibble1} to transmit                   (R/W)
//   0x0A TX_DATA1         {nibble4, nibble3}                               (R/W)
//   0x0B TX_DATA2         {nibble6, nibble5}                               (R/W)
//   0x0C TX_CONFIG        {2'b0, num_nibbles_tx[2:0], sync_mode,
//                          invert_tx, tx_enable}                          (R/W)
//                        tx_enable:  1 = transmit frames back-to-back
//                        invert_tx:  1 = invert uo_out[4]
//                        sync_mode:  1 = pad each frame to TX_FRAME_DUR ticks
//                                    with a trailing pause pulse
//                        num_nibbles_tx: 1-6 data nibbles per frame (0 or >6 -> 6)
//   0x0D TX_TICK          tick_cycles_tx[7:0]  (clk cycles per SENT tick)   (R/W)
//   0x0E TX_FRAME_DUR_L   frame_duration_ticks[7:0] (sync mode only)       (R/W)
//   0x0F TX_FRAME_DUR_H   {7'b0, frame_duration_ticks[8]} (9-bit value)     (R/W)
//                        (if TX_FRAME_DUR is shorter than the actual payload
//                        in sync mode, the pause is silently clamped to its
//                        minimum instead of going negative -- no error flag.
//                        tx_busy is not duplicated as an I2C register: it's
//                        already live on uo_out[5])
//   0x10 SYNC_MIN_L       sync_min_cycles[7:0]                              (R/W)
//   0x11 SYNC_MIN_H       sync_min_cycles[15:8]                             (R/W)
//   0x12 SYNC_MAX_L       sync_max_cycles[7:0]                              (R/W)
//   0x13 SYNC_MAX_H       sync_max_cycles[15:8]                             (R/W)
//                        bound the raw clock-cycle length of a plausible RX
//                        sync pulse (see sent_decoder.v); power up at the
//                        SYNC_MIN_CYCLES/SYNC_MAX_CYCLES module parameters
//                        below, then can be retuned at runtime for a
//                        different sensor tick length without recompiling
//
// STATUS_NIBBLE/DATA0-2/CRC are published from a shadow snapshot that only
// updates when no I2C transaction is in progress, so a multi-byte read is
// never torn between two different SENT frames. Symmetrically, the TX
// payload/config is latched into the encoder once per frame (at sync start),
// so a mid-frame I2C write never corrupts a transmission in progress.
//
module tt_um_enzonappi_sent_i2c #(
    // Power-on defaults for the SYNC_MIN_L/H, SYNC_MAX_L/H registers (0x10-0x13)
    // below -- NOT compile-time constants fed to the decoder any more, just the
    // reset value of a runtime-writable register pair. Defaults assume the TT
    // harness clock (10 MHz) and a 1-6 us SENT tick. Override for a different
    // system clock (e.g. an FPGA test board) or a sensor with a different
    // out-of-reset tick length -- see sent_decoder.v for the formula.
    parameter [15:0] SYNC_MIN_CYCLES = 16'd560,
    parameter [15:0] SYNC_MAX_CYCLES = 16'd3360
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    localparam integer DATA_NIBBLES = 6;
    localparam integer NUM_REGS     = 20;
    localparam [6:0]   I2C_ADDR     = 7'h50;
    // 9 bits covers frame_duration_ticks up to 511 ticks (>272, the longest
    // possible 6-nibble frame) with some headroom left for a pause
    localparam integer TICK_WIDTH   = 9;
    // 8 bits covers tick_cycles up to 255 clk cycles/tick: >60 needed for a
    // 6us tick at the ASIC's 10MHz clock; the DE10-Nano FPGA test (50MHz) can
    // reach up to ~5.1us/tick this way, just short of the full 6us bound
    localparam integer CYCLE_WIDTH  = 8;
    // sensible power-on tick length for TX so it never hangs unconfigured:
    // reuses the same 1 us/tick assumption as SYNC_MIN_CYCLES (56 ticks)
    localparam [CYCLE_WIDTH-1:0] DEFAULT_TICK_CYCLES_TX = (SYNC_MIN_CYCLES / 16'd56);

    // ---------------------------------------------------------------
    // Runtime configuration (CONFIG register, 0x07)
    // ---------------------------------------------------------------
    reg       invert_sent_cfg;
    reg       use_pause_cfg;
    reg [2:0] num_nibbles_cfg;

    // ---------------------------------------------------------------
    // Runtime configuration (SYNC_MIN/MAX registers, 0x10-0x13)
    // ---------------------------------------------------------------
    reg [15:0] sync_min_cycles_cfg;
    reg [15:0] sync_max_cycles_cfg;

    // ---------------------------------------------------------------
    // Runtime configuration (TX registers, 0x08-0x0F)
    // ---------------------------------------------------------------
    reg [3:0]              tx_status_nibble_r;
    reg [4*DATA_NIBBLES-1:0] tx_data_nibbles_r;
    reg                     tx_enable_cfg;
    reg                     invert_tx_cfg;
    reg                     sync_mode_cfg;
    reg [2:0]               num_nibbles_tx_cfg;
    reg [CYCLE_WIDTH-1:0]   tick_cycles_tx;
    reg [TICK_WIDTH-1:0]    frame_duration_ticks;

    // ---------------------------------------------------------------
    // SENT decoder
    // ---------------------------------------------------------------
    wire                      frame_valid;
    wire [3:0]                status_nibble;
    wire [4*DATA_NIBBLES-1:0] data_nibbles;
    wire [3:0]                crc_nibble;
    wire                      crc_error;
    wire                      sync_error;
    wire                      frame_busy;

    sent_decoder #(
        .DATA_NIBBLES     (DATA_NIBBLES)
    ) u_sent_decoder (
        .clk             (clk),
        .rst_n           (rst_n),
        .sent_in         (ui_in[0]),
        .invert_sent     (invert_sent_cfg),
        .use_pause       (use_pause_cfg),
        .num_nibbles_cfg (num_nibbles_cfg),
        .sync_min_cycles (sync_min_cycles_cfg),
        .sync_max_cycles (sync_max_cycles_cfg),
        .frame_valid     (frame_valid),
        .status_nibble   (status_nibble),
        .data_nibbles    (data_nibbles),
        .crc_nibble      (crc_nibble),
        .crc_error       (crc_error),
        .sync_error      (sync_error),
        .frame_busy      (frame_busy)
    );

    // ---------------------------------------------------------------
    // SENT encoder (independent of the decoder above -- true full duplex)
    // ---------------------------------------------------------------
    wire sent_tx_out;
    wire tx_busy;

    sent_encoder #(
        .DATA_NIBBLES (DATA_NIBBLES),
        .TICK_WIDTH   (TICK_WIDTH),
        .CYCLE_WIDTH  (CYCLE_WIDTH)
    ) u_sent_encoder (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .tx_enable             (tx_enable_cfg),
        .invert_tx             (invert_tx_cfg),
        .sync_mode             (sync_mode_cfg),
        .num_nibbles_cfg       (num_nibbles_tx_cfg),
        .status_nibble_tx      (tx_status_nibble_r),
        .data_nibbles_tx       (tx_data_nibbles_r),
        .tick_cycles           (tick_cycles_tx),
        .frame_duration_ticks  (frame_duration_ticks),
        .sent_out              (sent_tx_out),
        .tx_busy               (tx_busy)
    );

    // ---------------------------------------------------------------
    // I2C slave
    // ---------------------------------------------------------------
    wire       sda_oe;
    wire [4:0] reg_addr;
    reg  [7:0] reg_rdata;
    wire       read_strobe;
    wire [7:0] reg_wdata;
    wire       reg_wr;
    wire       i2c_busy;

    i2c_slave #(
        .I2C_ADDR (I2C_ADDR),
        .NUM_REGS (NUM_REGS)
    ) u_i2c_slave (
        .clk         (clk),
        .rst_n       (rst_n),
        .scl_in      (uio_in[0]),
        .sda_in      (uio_in[1]),
        .sda_oe      (sda_oe),
        .reg_addr    (reg_addr),
        .reg_rdata   (reg_rdata),
        .read_strobe (read_strobe),
        .reg_wdata   (reg_wdata),
        .reg_wr      (reg_wr),
        .busy        (i2c_busy)
    );

    // ---------------------------------------------------------------
    // CONFIG register write
    // ---------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            invert_sent_cfg <= 1'b0;
            use_pause_cfg   <= 1'b0;
            num_nibbles_cfg <= 3'd6;
        end else if (reg_wr && reg_addr == 5'd7) begin
            invert_sent_cfg <= reg_wdata[0];
            use_pause_cfg   <= reg_wdata[1];
            num_nibbles_cfg <= reg_wdata[4:2];
        end
    end

    // ---------------------------------------------------------------
    // SYNC_MIN/MAX register writes (0x10-0x13)
    // ---------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_min_cycles_cfg <= SYNC_MIN_CYCLES;
            sync_max_cycles_cfg <= SYNC_MAX_CYCLES;
        end else if (reg_wr) begin
            case (reg_addr)
                5'd16: sync_min_cycles_cfg[7:0]  <= reg_wdata;
                5'd17: sync_min_cycles_cfg[15:8] <= reg_wdata;
                5'd18: sync_max_cycles_cfg[7:0]  <= reg_wdata;
                5'd19: sync_max_cycles_cfg[15:8] <= reg_wdata;
                default: ; // not one of ours
            endcase
        end
    end

    // ---------------------------------------------------------------
    // TX register writes (0x08-0x0F)
    // ---------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_status_nibble_r   <= 4'd0;
            tx_data_nibbles_r    <= {(4*DATA_NIBBLES){1'b0}};
            tx_enable_cfg        <= 1'b0;
            invert_tx_cfg        <= 1'b0;
            sync_mode_cfg        <= 1'b0;
            num_nibbles_tx_cfg   <= 3'd6;
            tick_cycles_tx       <= DEFAULT_TICK_CYCLES_TX;
            frame_duration_ticks <= {TICK_WIDTH{1'b0}};
        end else if (reg_wr) begin
            case (reg_addr)
                5'd8:  tx_status_nibble_r      <= reg_wdata[3:0];
                5'd9:  tx_data_nibbles_r[7:0]   <= reg_wdata;
                5'd10: tx_data_nibbles_r[15:8]  <= reg_wdata;
                5'd11: tx_data_nibbles_r[23:16] <= reg_wdata;
                5'd12: begin
                    tx_enable_cfg      <= reg_wdata[0];
                    invert_tx_cfg      <= reg_wdata[1];
                    sync_mode_cfg      <= reg_wdata[2];
                    num_nibbles_tx_cfg <= reg_wdata[5:3];
                end
                5'd13: tick_cycles_tx             <= reg_wdata[CYCLE_WIDTH-1:0];
                5'd14: frame_duration_ticks[7:0]  <= reg_wdata;
                5'd15: frame_duration_ticks[TICK_WIDTH-1:8] <= reg_wdata[TICK_WIDTH-9:0];
                default: ; // 0x00-0x07 and 0x10-0x13 writes handled elsewhere / ignored
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Shadow snapshot of the decoded frame: only refreshed when no I2C
    // transaction is in progress, so a multi-byte read always sees a
    // single consistent frame instead of a mix of old/new data.
    // ---------------------------------------------------------------
    reg [3:0]                status_nibble_pub;
    reg [4*DATA_NIBBLES-1:0] data_nibbles_pub;
    reg [3:0]                crc_nibble_pub;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            status_nibble_pub <= 4'd0;
            data_nibbles_pub  <= {(4*DATA_NIBBLES){1'b0}};
            crc_nibble_pub    <= 4'd0;
        end else if (frame_valid && !i2c_busy) begin
            status_nibble_pub <= status_nibble;
            data_nibbles_pub  <= data_nibbles;
            crc_nibble_pub    <= crc_nibble;
        end
    end

    // ---------------------------------------------------------------
    // Sticky status bits + frame counter
    // ---------------------------------------------------------------
    reg new_data_sticky;
    reg crc_err_sticky;
    reg sync_err_sticky;
    reg [7:0] frame_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            new_data_sticky <= 1'b0;
            crc_err_sticky  <= 1'b0;
            sync_err_sticky <= 1'b0;
            frame_count     <= 8'd0;
        end else begin
            if (frame_valid) begin
                new_data_sticky <= 1'b1;
                frame_count     <= frame_count + 1'b1;
            end
            if (crc_error)  crc_err_sticky  <= 1'b1;
            if (sync_error) sync_err_sticky <= 1'b1;

            if (read_strobe && reg_addr == 5'd0) begin
                new_data_sticky <= 1'b0;
                crc_err_sticky  <= 1'b0;
                sync_err_sticky <= 1'b0;
            end
        end
    end

    // ---------------------------------------------------------------
    // Register file read mux
    // ---------------------------------------------------------------
    always @(*) begin
        case (reg_addr)
            5'd0: reg_rdata = {4'b0, frame_busy, sync_err_sticky, crc_err_sticky, new_data_sticky};
            5'd1: reg_rdata = {4'b0, status_nibble_pub};
            5'd2: reg_rdata = data_nibbles_pub[7:0];
            5'd3: reg_rdata = data_nibbles_pub[15:8];
            5'd4: reg_rdata = data_nibbles_pub[23:16];
            5'd5: reg_rdata = {4'b0, crc_nibble_pub};
            5'd6: reg_rdata = frame_count;
            5'd7: reg_rdata = {3'b0, num_nibbles_cfg, use_pause_cfg, invert_sent_cfg};
            5'd8: reg_rdata = {4'b0, tx_status_nibble_r};
            5'd9: reg_rdata = tx_data_nibbles_r[7:0];
            5'd10: reg_rdata = tx_data_nibbles_r[15:8];
            5'd11: reg_rdata = tx_data_nibbles_r[23:16];
            5'd12: reg_rdata = {2'b0, num_nibbles_tx_cfg, sync_mode_cfg, invert_tx_cfg, tx_enable_cfg};
            5'd13: reg_rdata = {{(8-CYCLE_WIDTH){1'b0}}, tick_cycles_tx};
            5'd14: reg_rdata = frame_duration_ticks[7:0];
            5'd15: reg_rdata = {{(8-(TICK_WIDTH-8)){1'b0}}, frame_duration_ticks[TICK_WIDTH-1:8]};
            5'd16: reg_rdata = sync_min_cycles_cfg[7:0];
            5'd17: reg_rdata = sync_min_cycles_cfg[15:8];
            5'd18: reg_rdata = sync_max_cycles_cfg[7:0];
            5'd19: reg_rdata = sync_max_cycles_cfg[15:8];
            default: reg_rdata = 8'h00;
        endcase
    end

    // ---------------------------------------------------------------
    // TT pin wiring
    // ---------------------------------------------------------------
    assign uo_out = {2'b0, tx_busy, sent_tx_out, frame_busy, sync_err_sticky, crc_err_sticky, new_data_sticky};

    assign uio_out = {6'b0, 1'b0, 1'b0}; // open-drain: never actively drive high
    assign uio_oe  = {6'b0, sda_oe, 1'b0}; // uio[1]=SDA oe, uio[0]=SCL always input

    // silence unused-signal lint warnings without affecting synthesis
    wire _unused_ok = &{ena, ui_in[7:1], uio_in[7:2], 1'b0};

endmodule

`default_nettype wire
