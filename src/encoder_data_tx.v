module encoder_data_tx (
    input wire clk,
    input wire rst_n,
    input wire transmit_enable,        // Enable periodic transmission
    input wire signed [15:0] enc0_pos,
    input wire signed [15:0] enc1_pos,
    input wire signed [15:0] rpm0,
    input wire signed [15:0] rpm1,
    input wire enc0_dir,
    input wire enc1_dir,
    input wire uart_tx_busy,
    output reg [7:0] uart_tx_data,
    output reg uart_tx_valid
);

    // CORRECTED transmission protocol to match Python GUI (9 bytes):
    // Byte 0:   Header (0xFF)
    // Byte 1-2: Motor 0 position (16-bit signed, big endian)
    // Byte 3-4: Motor 0 RPM (16-bit signed, big endian)
    // Byte 5-6: Motor 1 position (16-bit signed, big endian)
    // Byte 7-8: Motor 1 RPM (16-bit signed, big endian)
    
    localparam CLK_FREQ = 100_000_000;
    localparam TX_RATE = 3;  // Further reduce to 3Hz for ultimate reliability
    localparam TX_INTERVAL = CLK_FREQ / TX_RATE;
    
    reg [31:0] tx_timer = 0;
    reg [3:0] tx_state = 0;
    reg tx_active = 0;
    reg [7:0] tx_counter = 0;
    
    // Simple state machine
    localparam IDLE = 4'd0;
    localparam SEND_BYTE = 4'd1;
    localparam WAIT_BUSY = 4'd2;
    
    // Data array for transmission
    reg [7:0] tx_data_array [0:8];
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_timer <= 0;
            tx_state <= IDLE;
            tx_active <= 0;
            uart_tx_data <= 0;
            uart_tx_valid <= 0;
            tx_counter <= 0;
        end else begin
            uart_tx_valid <= 0;
            
            if (transmit_enable) begin
                // Timer for periodic transmission - ADD DELAY after command
                if (!tx_active) begin
                    if (tx_timer >= TX_INTERVAL - 1) begin
                        tx_timer <= 0;
                        // WAIT for UART to be completely free before starting encoder transmission
                        // Add extra safety checks to prevent collision with command responses
                        if (!uart_tx_busy) begin
                            // Prepare data array - CORRECTED format to match Python GUI
                            tx_data_array[0] <= 8'hFF;           // Header
                            tx_data_array[1] <= enc0_pos[15:8];  // Motor 0 pos high (big endian)
                            tx_data_array[2] <= enc0_pos[7:0];   // Motor 0 pos low
                            tx_data_array[3] <= rpm0[15:8];      // Motor 0 RPM high (big endian)
                            tx_data_array[4] <= rpm0[7:0];       // Motor 0 RPM low
                            tx_data_array[5] <= enc1_pos[15:8];  // Motor 1 pos high (big endian)
                            tx_data_array[6] <= enc1_pos[7:0];   // Motor 1 pos low
                            tx_data_array[7] <= rpm1[15:8];      // Motor 1 RPM high (big endian)
                            tx_data_array[8] <= rpm1[7:0];       // Motor 1 RPM low
                            
                            tx_active <= 1;
                            tx_state <= SEND_BYTE;
                            tx_counter <= 0;
                        end
                    end else begin
                        tx_timer <= tx_timer + 1;
                    end
                end
                
                // Simple transmission state machine
                if (tx_active) begin
                    case (tx_state)
                        SEND_BYTE: begin
                            // WAIT for UART to be completely ready
                            if (!uart_tx_busy) begin
                                uart_tx_data <= tx_data_array[tx_counter];
                                uart_tx_valid <= 1;
                                tx_state <= WAIT_BUSY;
                            end
                            // Timeout protection - if stuck, reset
                            else if (tx_counter > 20) begin  // Safety timeout
                                tx_active <= 0;
                                tx_state <= IDLE;
                                tx_counter <= 0;
                            end
                        end
                        
                        WAIT_BUSY: begin
                            if (uart_tx_busy) begin
                                // UART started sending, now wait for completion
                                tx_state <= SEND_BYTE;
                                if (tx_counter >= 8) begin
                                    // All bytes sent
                                    tx_active <= 0;
                                    tx_state <= IDLE;
                                    tx_counter <= 0;
                                end else begin
                                    tx_counter <= tx_counter + 1;
                                end
                            end
                        end
                        
                        default: begin
                            tx_active <= 0;
                            tx_state <= IDLE;
                            tx_counter <= 0;
                        end
                    endcase
                end
            end else begin
                // When transmission disabled, FORCE reset all states
                tx_timer <= 0;
                tx_active <= 0;
                tx_state <= IDLE;
                tx_counter <= 0;
                uart_tx_valid <= 0;  // Ensure no lingering valid signals
            end
        end
    end

endmodule
