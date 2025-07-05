//**********************************************************************
//  Project: Motor Control System
//  File: pid_controller.v
//  Description: PID Controller for Speed (RPM) Control
//  Purpose: Calculate PWM output based on RPM setpoint and feedback
//----------------------------------------------------------------------
// PID Implementation:
// - Error = Setpoint - Feedback
// - Proportional = Kp × Error
// - Integral += Ki × Error × dt
// - Derivative = Kd × (Error - Previous_Error) / dt
// - Output = P + I + D (clamped to PWM limits)
//**********************************************************************

module pid_controller (
    input wire clk,
    input wire rst_n,
    
    // PID Parameters
    input wire signed [15:0] setpoint,  // Target RPM (signed for negative RPM)
    input wire [15:0] kp,            // Proportional gain (scaled by 256)
    input wire [15:0] ki,            // Integral gain (scaled by 256)
    input wire [15:0] kd,            // Derivative gain (scaled by 256)
    
    // Feedback and control
    input wire signed [15:0] feedback, // Current RPM from encoder
    input wire feedback_valid,         // RPM data valid flag
    input wire pid_enable,             // Enable PID control
    input wire [15:0] max_output,      // Maximum PWM output (CCR value)
    
    // Output
    output reg signed [15:0] pid_output,
    output reg output_valid
);

//**********************************************************************
// --- PID Variables and Parameters
//**********************************************************************

    // PID calculation variables
    reg signed [31:0] error = 0;
    reg signed [31:0] error_prev = 0;
    reg signed [31:0] integral = 0;
    reg signed [31:0] derivative = 0;
    
    // PID terms
    reg signed [31:0] proportional = 0;
    reg signed [31:0] pid_sum = 0;
    
    // Timing for PID calculation
    reg [31:0] pid_timer = 0;
    localparam PID_PERIOD = 1_000_000;  // 10ms at 100MHz (100Hz PID rate)
    
    // Integral limits for anti-windup
    localparam INTEGRAL_MAX = 32767 * 256;   // Prevent integral windup
    localparam INTEGRAL_MIN = -32767 * 256;

//**********************************************************************
// --- PID Calculation - Main Logic
//**********************************************************************

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error <= 0;
            error_prev <= 0;
            integral <= 0;
            derivative <= 0;
            proportional <= 0;
            pid_sum <= 0;
            pid_output <= 0;
            output_valid <= 0;
            pid_timer <= 0;
        end else begin
            output_valid <= 0;
            pid_timer <= pid_timer + 1;
            
            // PID calculation every 10ms (100Hz rate)
            if (pid_timer >= PID_PERIOD && pid_enable && feedback_valid) begin
                pid_timer <= 0;
                
                // Calculate error
                // error <= setpoint - feedback;
                // Sign-extend setpoint and feedback to 32 bits before subtraction
                error <= $signed({{16{setpoint[15]}}, setpoint}) - $signed({{16{feedback[15]}}, feedback});
                
                // Reset integral and output when setpoint is zero (motor disabled)
                if (setpoint == 0) begin
                    integral <= 0;
                    proportional <= 0;
                    derivative <= 0;
                    pid_sum <= 0;
                    pid_output <= 0;
                    error_prev <= 0;
                end else begin
                    // Proportional term: P = Kp × Error
                    proportional <= ($signed(kp) * error) >>> 8;  // Divide by 256 (scaling)
                    
                    // Integral term: I += Ki × Error × dt
                    if (integral + (($signed(ki) * error) >>> 8) > INTEGRAL_MAX) begin
                        integral <= INTEGRAL_MAX;  // Anti-windup
                    end else if (integral + (($signed(ki) * error) >>> 8) < INTEGRAL_MIN) begin
                        integral <= INTEGRAL_MIN;  // Anti-windup
                    end else begin
                        integral <= integral + (($signed(ki) * error) >>> 8);
                    end
                end
                
                // Derivative term and PID sum calculation (only when setpoint != 0)
                if (setpoint != 0) begin
                    // Derivative term: D = Kd × (Error - Error_prev) / dt
                    derivative <= ($signed(kd) * (error - error_prev)) >>> 8;
                    
                    // Calculate PID output: Output = P + I + D
                    pid_sum <= proportional + integral + derivative;
                    
                    // Clamp output to valid PWM range
                    if (pid_sum > $signed(max_output)) begin
                        pid_output <= $signed(max_output);
                    end else if (pid_sum < -$signed(max_output)) begin
                        pid_output <= -$signed(max_output);
                    end else begin
                        pid_output <= pid_sum[15:0];
                    end
                end
                
                // Update previous error for next derivative calculation (only when setpoint != 0)
                if (setpoint != 0) begin
                    error_prev <= error;
                end
                output_valid <= 1;
            end
            
            // Reset integral when PID is disabled to prevent windup
            if (!pid_enable) begin
                integral <= 0;
                error_prev <= 0;
                pid_output <= 0;
            end
        end
    end

endmodule
