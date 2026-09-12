`default_nettype none
`timescale 1ns / 1ps

// Sequential shift-subtract (restoring) unsigned divider.
// Latency: WIDTH clock cycles. Shared by the SENT decoder for both
// tick-length calibration (sync_period / TICKS_SYNC) and per-nibble
// period-to-ticks conversion (period / tick_period), so it only needs
// to be instantiated once.
module divider #(
    parameter WIDTH = 16
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             start,      // pulse to load dividend/divisor and begin
    input  wire [WIDTH-1:0] dividend,
    input  wire [WIDTH-1:0] divisor,
    output reg  [WIDTH-1:0] quotient,
    output reg  [WIDTH-1:0] remainder,
    output reg              busy,
    output reg              done        // 1-cycle pulse when quotient/remainder are valid
);

    reg [WIDTH-1:0] q;
    reg [WIDTH-1:0] r;
    reg [$clog2(WIDTH+1)-1:0] count;

    wire [WIDTH-1:0] r_shift = {r[WIDTH-2:0], q[WIDTH-1]};
    wire             sub_ok  = (r_shift >= divisor);
    wire [WIDTH-1:0] r_next  = sub_ok ? (r_shift - divisor) : r_shift;
    wire [WIDTH-1:0] q_next  = {q[WIDTH-2:0], sub_ok};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            quotient  <= {WIDTH{1'b0}};
            remainder <= {WIDTH{1'b0}};
            q         <= {WIDTH{1'b0}};
            r         <= {WIDTH{1'b0}};
            count     <= {$clog2(WIDTH+1){1'b0}};
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy  <= 1'b1;
                q     <= dividend;
                r     <= {WIDTH{1'b0}};
                count <= {$clog2(WIDTH+1){1'b0}};
            end else if (busy) begin
                q <= q_next;
                r <= r_next;
                if (count == WIDTH - 1) begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    quotient  <= q_next;
                    remainder <= r_next;
                end else begin
                    count <= count + 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
