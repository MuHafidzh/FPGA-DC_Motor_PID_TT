module uart_rx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD     = 115200
)(
    input  wire clk,
    input  wire rst_n,
    input  wire rx,
    output reg  [7:0] data,
    output reg  data_valid
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
    reg [7:0] rx_byte = 0;
    
    // Double flop RX input for metastability
    reg rx_d1 = 1, rx_d2 = 1;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_d1 <= 1;
            rx_d2 <= 1;
        end else begin
            rx_d1 <= rx;
            rx_d2 <= rx_d1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            clk_cnt <= 0;
            bit_idx <= 0;
            rx_byte <= 0;
            data <= 0;
            data_valid <= 0;
        end else begin
            data_valid <= 0;
            
            case (state)
                IDLE: begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    if (rx_d2 == 0) begin  // Start bit
                        state <= START;
                    end
                end
                
                START: begin
                    if (clk_cnt == (CLKS_PER_BIT/2)) begin
                        if (rx_d2 == 0) begin  // Valid start bit
                            clk_cnt <= 0;
                            state <= DATA;
                        end else begin
                            state <= IDLE;  // False start
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
                
                DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= 0;
                        rx_byte[bit_idx] <= rx_d2;
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
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        state <= IDLE;
                        data <= rx_byte;
                        data_valid <= 1;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end
            endcase
        end
    end
endmodule