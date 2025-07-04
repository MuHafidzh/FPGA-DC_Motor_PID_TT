module uart_tx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD     = 115200
)(
    input  wire clk,
    input  wire rst_n,
    input  wire [7:0] data,
    input  wire data_valid,
    output reg  tx,
    output reg  busy
);
    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD;
    
    // State machine
    localparam IDLE  = 2'b00;
    localparam START = 2'b01;
    localparam DATA  = 2'b10;
    localparam STOP  = 2'b11;
    
    reg [1:0] state = IDLE;
    reg [15:0] clk_cnt = 0;
    reg [2:0] bit_idx = 0;
    reg [7:0] tx_byte = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            clk_cnt <= 0;
            bit_idx <= 0;
            tx_byte <= 0;
            tx <= 1;
            busy <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1;
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    busy <= 0;
                    if (data_valid) begin
                        tx_byte <= data;
                        state <= START;
                        busy <= 1;
                    end
                end
                
                START: begin
                    tx <= 0;  // Start bit
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        state <= DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
                
                DATA: begin
                    tx <= tx_byte[bit_idx];
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        if (bit_idx == 7) begin
                            bit_idx <= 0;
                            state <= STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
                
                STOP: begin
                    tx <= 1;  // Stop bit
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        state <= IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end
endmodule