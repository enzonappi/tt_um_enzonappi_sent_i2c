`default_nettype none
`timescale 1ns / 1ps

// Minimal I2C slave with a register-map interface (like a typical sensor):
// - master WRITEs 1 byte to set the register pointer, optionally followed
//   by 1 more byte to write into that register (for writable config regs)
// - master READs 1..N bytes starting at that pointer, auto-incrementing
//   (wrapping at NUM_REGS) for as long as it keeps ACKing
// No clock stretching: SCL is treated as an input only.
module i2c_slave #(
    parameter [6:0]  I2C_ADDR = 7'h50,
    parameter integer NUM_REGS = 7   // must fit in 5 bits (<=32)
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       scl_in,
    input  wire       sda_in,
    output reg        sda_oe,       // 1 = actively pull SDA low, 0 = release (Hi-Z)

    output reg  [4:0] reg_addr,     // current register pointer
    input  wire [7:0] reg_rdata,    // combinational data for reg_addr, from the top-level reg file
    output reg        read_strobe,  // 1-cycle pulse: the reg_addr byte has just been shifted out
    output reg  [7:0] reg_wdata,    // byte written by the master after the pointer
    output reg        reg_wr,       // 1-cycle pulse: reg_wdata is valid for reg_addr
    output wire       busy          // 1 while a transaction (START..STOP) is in progress
);

    // ---------------------------------------------------------------
    // Synchronizers + start/stop/edge detection
    // ---------------------------------------------------------------
    reg [1:0] scl_sync, sda_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            scl_sync <= {scl_sync[0], scl_in};
            sda_sync <= {sda_sync[0], sda_in};
        end
    end
    wire scl_s = scl_sync[1];
    wire sda_s = sda_sync[1];

    reg scl_prev, sda_prev, sda_prev2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_prev  <= 1'b1;
            sda_prev  <= 1'b1;
            sda_prev2 <= 1'b1;
        end else begin
            scl_prev  <= scl_s;
            sda_prev2 <= sda_prev;
            sda_prev  <= sda_s;
        end
    end

    wire scl_rise   = scl_s  & ~scl_prev;
    wire scl_fall   = ~scl_s &  scl_prev;
    // require SDA to hold its new level for 2 consecutive samples before
    // accepting a START/STOP: a single-cycle read can catch SDA mid-swing
    // (the bus's own pull-up/pull-down RC transition time, not a real
    // glitch) and misread it as an edge -- especially right after we
    // release SDA for a '1' data bit close to the next SCL rising edge.
    // Costs one extra clk of detection latency (20ns @ 50MHz), negligible
    // next to real I2C bit timing (>=2.5us/bit at 400kHz). Bit sampling
    // (scl_rise/scl_fall above) is untouched, so this doesn't slow down
    // normal data transfer.
    wire start_cond = scl_s  &  sda_prev2 & ~sda_prev & ~sda_s; // SDA fell and stayed low
    wire stop_cond  = scl_s  & ~sda_prev2 &  sda_prev &  sda_s; // SDA rose and stayed high

    // ---------------------------------------------------------------
    // Main FSM
    // ---------------------------------------------------------------
    localparam [3:0]
        IDLE          = 4'd0,
        ADDR          = 4'd1,
        ADDR_ACK      = 4'd2,
        REGPTR        = 4'd3,
        REGPTR_ACK    = 4'd4,
        RDATA         = 4'd5,
        RACK          = 4'd6,
        RELOAD        = 4'd7,
        WRDATA        = 4'd8,
        WRDATA_ACK    = 4'd9,
        WRDATA_ACK_REL = 4'd10;

    reg [3:0] state;
    reg [2:0] bitcnt;
    reg [7:0] shreg_in;
    reg [7:0] shreg_out;

    assign busy = (state != IDLE);

    wire [4:0] ptr_next = (reg_addr == NUM_REGS - 1) ? 5'd0 : (reg_addr + 5'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            bitcnt      <= 3'd0;
            shreg_in    <= 8'd0;
            shreg_out   <= 8'd0;
            sda_oe      <= 1'b0;
            reg_addr    <= 5'd0;
            read_strobe <= 1'b0;
            reg_wdata   <= 8'd0;
            reg_wr      <= 1'b0;
        end else begin
            read_strobe <= 1'b0;
            reg_wr      <= 1'b0;

            if (start_cond) begin
                state  <= ADDR;
                bitcnt <= 3'd0;
                sda_oe <= 1'b0;
            end else if (stop_cond) begin
                state  <= IDLE;
                sda_oe <= 1'b0;
            end else begin
                case (state)

                    IDLE: begin
                        sda_oe <= 1'b0;
                    end

                    // ---- receive address + R/W bit ----
                    ADDR: begin
                        if (scl_rise) begin
                            shreg_in <= {shreg_in[6:0], sda_s};
                            if (bitcnt == 3'd7) state <= ADDR_ACK;
                            bitcnt <= bitcnt + 3'd1;
                        end
                    end

                    ADDR_ACK: begin
                        if (scl_fall) begin
                            if (shreg_in[7:1] == I2C_ADDR) sda_oe <= 1'b1; // ACK
                            else                           sda_oe <= 1'b0; // NACK: no match
                        end
                        if (scl_rise) begin
                            if (shreg_in[7:1] == I2C_ADDR) begin
                                if (shreg_in[0]) begin // read
                                    shreg_out <= reg_rdata;
                                    state     <= RDATA;
                                    bitcnt    <= 3'd0;
                                end else begin // write: expect a pointer byte
                                    state  <= REGPTR;
                                    bitcnt <= 3'd0;
                                end
                            end else begin
                                state <= IDLE;
                            end
                        end
                    end

                    // ---- write path: receive the new register pointer ----
                    REGPTR: begin
                        if (scl_fall) sda_oe <= 1'b0;
                        if (scl_rise) begin
                            shreg_in <= {shreg_in[6:0], sda_s};
                            if (bitcnt == 3'd7) state <= REGPTR_ACK;
                            bitcnt <= bitcnt + 3'd1;
                        end
                    end

                    REGPTR_ACK: begin
                        if (scl_fall) sda_oe <= 1'b1; // ACK the pointer byte
                        if (scl_rise) begin
                            reg_addr <= (shreg_in >= NUM_REGS) ? 5'd0 : shreg_in[4:0];
                            state    <= WRDATA; // master may STOP here, or send 1 data byte
                            bitcnt   <= 3'd0;
                        end
                    end

                    // ---- write path: optional data byte for the pointed-at register ----
                    WRDATA: begin
                        if (scl_fall) sda_oe <= 1'b0;
                        if (scl_rise) begin
                            shreg_in <= {shreg_in[6:0], sda_s};
                            if (bitcnt == 3'd7) state <= WRDATA_ACK;
                            bitcnt <= bitcnt + 3'd1;
                        end
                    end

                    WRDATA_ACK: begin
                        if (scl_fall) sda_oe <= 1'b1; // ACK the data byte
                        if (scl_rise) begin
                            reg_wdata <= shreg_in;
                            reg_wr    <= 1'b1;
                            // don't go straight to IDLE: IDLE releases sda_oe
                            // unconditionally (every cycle), which would fire
                            // on the very next clk tick -- almost certainly
                            // still within this ACK bit's SCL-high phase --
                            // and yank SDA high while SCL is high, looking
                            // like a spurious STOP to the master. Wait one
                            // more real scl_fall before releasing.
                            state <= WRDATA_ACK_REL;
                        end
                    end

                    WRDATA_ACK_REL: begin
                        if (scl_fall) begin
                            sda_oe <= 1'b0;
                            state  <= IDLE;
                        end
                    end

                    // ---- read path: shift out the current register ----
                    RDATA: begin
                        // re-assert every cycle SCL reads low (not just once
                        // on the scl_fall edge) so a single missed/glitched
                        // edge can't leave sda_oe stuck on a stale bit for
                        // the rest of the byte -- safe: SDA may change any
                        // number of times while SCL is low, only a change
                        // while SCL is high is protocol-illegal
                        if (!scl_s) sda_oe <= ~shreg_out[7];
                        if (scl_rise) begin
                            shreg_out <= {shreg_out[6:0], 1'b0};
                            if (bitcnt == 3'd7) begin
                                read_strobe <= 1'b1;
                                state       <= RACK;
                            end
                            bitcnt <= bitcnt + 3'd1;
                        end
                    end

                    RACK: begin
                        if (scl_fall) sda_oe <= 1'b0; // release: master drives ack/nack
                        if (scl_rise) begin
                            if (sda_s == 1'b0) begin // ACK: master wants more
                                reg_addr <= ptr_next;
                                state    <= RELOAD;
                            end else begin // NACK: master is done
                                state <= IDLE;
                            end
                        end
                    end

                    RELOAD: begin
                        // give reg_rdata (a wide combinational mux, one level
                        // deeper for later register addresses) several clk
                        // cycles to settle on the new reg_addr before we
                        // latch it -- one cycle is enough in RTL simulation
                        // (zero-delay combinational logic) but may not be on
                        // real silicon with real gate/routing delay. Still
                        // negligible next to real I2C bit timing (>=4 cycles
                        // = 80ns @ 50MHz vs >=2.5us/bit at 400kHz).
                        if (bitcnt == 3'd3) begin
                            shreg_out <= reg_rdata;
                            bitcnt    <= 3'd0;
                            state     <= RDATA;
                        end else begin
                            bitcnt <= bitcnt + 3'd1;
                        end
                    end

                    default: state <= IDLE;

                endcase
            end
        end
    end

endmodule

`default_nettype wire
