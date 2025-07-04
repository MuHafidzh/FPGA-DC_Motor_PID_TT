//**********************************************************************
//  Project: Motor Control System
//  File: quadrature_encoder.v
//  Description: SIMPLIFIED encoder with PPR-based RPM calculation
//  Purpose: Count encoder pulses and calculate accurate RPM using PPR
//----------------------------------------------------------------------
// Implementation:
// - Simple pulse counting (rising edge of channel A only)
// - Direction detection based on A/B phase relationship
// - RPM calculation: RPM = (pulse_count × 600) / PPR
//   where 600 = 60 sec/min × 10 (measurement window is 100ms = 0.1s)
// - PPR (Pulses Per Revolution) received from UART configuration
// Note: This is NOT a true quadrature encoder (only 1x resolution)
//**********************************************************************

module simple_encoder (
    input wire clk,
    input wire rst_n,
    input wire enc_a,
    input wire enc_b,
    input wire [15:0] ppr,  // Pulses per revolution from UART
    output reg signed [15:0] position,
    output reg direction,  // 1=forward, 0=reverse
    output reg signed [15:0] rpm,
    output reg rpm_valid
);

//**********************************************************************
// --- Simple Implementation - Just count rising edges (1x resolution)
//**********************************************************************
    // Simple 2-stage synchronizer
    reg [1:0] enc_a_sync = 2'b00;
    reg [1:0] enc_b_sync = 2'b00;
    
    // Previous values for edge detection
    reg enc_a_prev = 0;
    reg enc_b_prev = 0;
    
    // Simple counter for RPM estimation
    reg [31:0] pulse_counter = 0;
    reg [31:0] time_counter = 0;
    reg [31:0] idle_counter = 0;  // Counter for detecting motor stopped
    localparam TIME_WINDOW = 10_000_000;  // 100ms at 100MHz (faster update)
    localparam IDLE_TIMEOUT = 20_000_000;  // 200ms timeout to force RPM = 0

//**********************************************************************
// --- Main Logic - Keep it VERY simple
//**********************************************************************

    // Simple 2-stage synchronizer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_a_sync <= 2'b00;
            enc_b_sync <= 2'b00;
            enc_a_prev <= 0;
            enc_b_prev <= 0;
        end else begin
            // 2-stage sync
            enc_a_sync <= {enc_a_sync[0], enc_a};
            enc_b_sync <= {enc_b_sync[0], enc_b};
            
            // Store previous values
            enc_a_prev <= enc_a_sync[1];
            enc_b_prev <= enc_b_sync[1];
        end
    end

    // Simple position counter - count rising edges of enc_a
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position <= 0;
            direction <= 0;
        end else begin
            // Rising edge on enc_a
            if (!enc_a_prev && enc_a_sync[1]) begin
                // Check enc_b for direction - FLIP LOGIC to match motor direction
                if (!enc_b_sync[1]) begin
                    // Forward (PWM positive should give positive encoder)
                    if (position < 32767)
                        position <= position + 1;
                    direction <= 1;
                end else begin
                    // Reverse (PWM negative should give negative encoder)
                    if (position > -32768)
                        position <= position - 1;
                    direction <= 0;
                end
            end
        end
    end

    // Very simple RPM calculation - count pulses per second
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_counter <= 0;
            time_counter <= 0;
            idle_counter <= 0;
            rpm <= 0;
            rpm_valid <= 0;
        end else begin
            time_counter <= time_counter + 1;
            idle_counter <= idle_counter + 1;
            
            // Count pulses (rising edge of enc_a)
            if (!enc_a_prev && enc_a_sync[1]) begin
                pulse_counter <= pulse_counter + 1;
                idle_counter <= 0;  // Reset idle counter when there's movement
            end
            
            // Force RPM to 0 if no movement for too long
            if (idle_counter >= IDLE_TIMEOUT) begin
                rpm <= 0;
                rpm_valid <= 1;
                idle_counter <= 0;
            end
            
            // Every 100ms, calculate RPM (faster updates)
            if (time_counter >= TIME_WINDOW) begin
                // RPM calculation based on actual pulse count
                if (pulse_counter == 0) begin
                    // No pulses = no movement = RPM is 0
                    rpm <= 0;
                end else begin
                    // Real RPM calculation: RPM = (pulses × 600) / PPR
                    // 600 = 60 seconds/minute × 10 (because we measure in 100ms = 0.1s)
                    // RPM = (pulse_count × 60) / (0.1 × PPR) = (pulse_count × 600) / PPR
                    if (ppr > 0 && ppr <= 16383) begin  // Prevent division overflow (600*16383 = ~10M fits in 32-bit)
                        rpm <= (pulse_counter * 600) / ppr;
                    end else if (ppr > 16383) begin
                        // For very high PPR, use different scaling to prevent overflow
                        rpm <= (pulse_counter * 60) / (ppr / 10);
                    end else begin
                        rpm <= pulse_counter * 6;  // Fallback if PPR not set
                    end
                    
                    // Apply direction (only if there's actual movement)
                    if (!direction && rpm > 0)
                        rpm <= -rpm;
                end
                
                rpm_valid <= 1;
                
                // Reset counters
                pulse_counter <= 0;
                time_counter <= 0;
            end else begin
                rpm_valid <= 0;
            end
        end
    end

endmodule
