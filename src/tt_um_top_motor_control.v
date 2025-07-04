`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

module top_motor_control(
    input wire CLK100MHZ,
    input wire CPU_RESETN,
    // UART
    input wire UART_RX,
    output wire UART_TX,
    // PWM output
    output wire [1:0] PWM_OUT,
    // Motor direction signal
    output wire [3:0] MOTOR_IN,
    // LEDs for visualize the IN
    output wire [3:0] LED,
    // Encoder input
    input wire [1:0] ENC_A,
    input wire [1:0] ENC_B
);

    // UART byte-wise interface
    wire [7:0] uart_rx_data;
    wire uart_rx_valid;
    wire uart_tx_busy;

    // UART RX/TX (8-bit simple)
    uart_rx #(.CLK_FREQ(100_000_000), .BAUD(115200)) u_uart_rx (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .rx(UART_RX),
        .data(uart_rx_data),
        .data_valid(uart_rx_valid)
    );

    reg [7:0] uart_tx_data;
    reg uart_tx_valid;

    uart_tx #(.CLK_FREQ(100_000_000), .BAUD(115200)) u_uart_tx (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .data(uart_tx_data),
        .data_valid(uart_tx_valid),
        .tx(UART_TX),
        .busy(uart_tx_busy)
    );

    // Protocol: 
    // Motor Command: 3 bytes - [0xAA/0xBB][PWM_High][PWM_Low]
    // PSC Command: 3 bytes - [0xCC][PSC_High][PSC_Low] 
    // CCR Command: 3 bytes - [0xDD][CCR_High][CCR_Low]
    // PPR Command: 3 bytes - [0x11][PPR_High][PPR_Low]
    // PID Mode Command: 3 bytes - [0x22][Motor_ID][Mode] (Mode: 0=PWM, 1=PID)
    // PID Setpoint: 3 bytes - [0x33][Motor_ID][Setpoint_High][Setpoint_Low]
    // PID Kp: 3 bytes - [0x44][Motor_ID][Kp_High][Kp_Low]
    // PID Ki: 3 bytes - [0x55][Motor_ID][Ki_High][Ki_Low]
    // PID Kd: 3 bytes - [0x66][Motor_ID][Kd_High][Kd_Low]
    // Query Command: 1 byte - [0xEE] -> returns PSC, CCR, and PPR values
    // Note: Encoder data transmission is always enabled for real-time monitoring
    
    reg signed [15:0] pwm_reg0 = 0;
    reg signed [15:0] pwm_reg1 = 0;
    reg [15:0] psc_reg = 99;   // Default PSC = 99
    reg [15:0] ccr_reg = 999;  // Default CCR = 999
    reg [15:0] ppr_reg = 1100; // Default PPR = 1100
    
    // PID Control registers
    reg [1:0] pid_mode = 0;    // Bit 0 = Motor0 mode, Bit 1 = Motor1 mode (0=PWM, 1=PID)
    reg signed [15:0] pid_setpoint0 = 0;  // Target RPM for Motor 0 (signed for negative RPM)
    reg signed [15:0] pid_setpoint1 = 0;  // Target RPM for Motor 1 (signed for negative RPM)
    reg [15:0] pid_kp0 = 256;      // Kp for Motor 0 (scaled by 256, so 256 = 1.0)
    reg [15:0] pid_ki0 = 64;       // Ki for Motor 0
    reg [15:0] pid_kd0 = 16;       // Kd for Motor 0
    reg [15:0] pid_kp1 = 256;      // Kp for Motor 1
    reg [15:0] pid_ki1 = 64;       // Ki for Motor 1
    reg [15:0] pid_kd1 = 16;       // Kd for Motor 1
    
    reg [2:0] rx_state = 0;    // 0=wait header, 1=wait high byte, 2=wait low byte, 3=wait extra byte for 4-byte commands
    reg [15:0] temp_data = 0;
    reg [3:0] current_cmd = 0; // Extended to 4 bits for more commands
    reg [7:0] motor_id_temp = 0; // Temporary storage for motor ID in PID commands
    reg cmd_pending = 0;       // Flag to track pending command response
    
    // Clamp function (now uses dynamic CCR)
    function signed [15:0] clamp_pwm;
        input signed [15:0] pwm_in;
        begin
            if (pwm_in > $signed(ccr_reg))
                clamp_pwm = $signed(ccr_reg);
            else if (pwm_in < -$signed(ccr_reg))
                clamp_pwm = -$signed(ccr_reg);
            else
                clamp_pwm = pwm_in;
        end
    endfunction

    always @(posedge CLK100MHZ or negedge CPU_RESETN) begin
        if (!CPU_RESETN) begin
            pwm_reg0 <= 0;
            pwm_reg1 <= 0;
            psc_reg <= 99;
            ccr_reg <= 999;
            ppr_reg <= 1100;
            
            // PID reset values
            pid_mode <= 0;
            pid_setpoint0 <= 0;
            pid_setpoint1 <= 0;
            pid_kp0 <= 256;  // Default Kp = 1.0
            pid_ki0 <= 64;   // Default Ki = 0.25
            pid_kd0 <= 16;   // Default Kd = 0.0625
            pid_kp1 <= 256;
            pid_ki1 <= 64;
            pid_kd1 <= 16;
            
            rx_state <= 0;
            temp_data <= 0;
            current_cmd <= 0;
            motor_id_temp <= 0;
            cmd_tx_data <= 0;
            cmd_tx_valid <= 0;
            cmd_pending <= 0;
        end else begin
            // Clear cmd_tx_valid when UART transmission is accepted
            if (cmd_tx_valid && !uart_tx_busy) begin
                cmd_tx_valid <= 0;
                cmd_pending <= 0;
            end
            
            // ULTRA SIMPLIFIED: Process RX commands immediately
            if (uart_rx_valid) begin
                case (rx_state)
                    3'd0: begin  // Wait for header
                        if (uart_rx_data == 8'hAA) begin
                            current_cmd <= 0;  // Motor 0 PWM
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'hBB) begin
                            current_cmd <= 1;  // Motor 1 PWM
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'hCC) begin
                            current_cmd <= 2;  // PSC command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'hDD) begin
                            current_cmd <= 3;  // CCR command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h11) begin
                            current_cmd <= 4;  // PPR command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h22) begin
                            current_cmd <= 5;  // PID Mode command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h33) begin
                            current_cmd <= 6;  // PID Setpoint command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h44) begin
                            current_cmd <= 7;  // PID Kp command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h55) begin
                            current_cmd <= 8;  // PID Ki command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'h66) begin
                            current_cmd <= 9;  // PID Kd command
                            rx_state <= 1;
                        end else if (uart_rx_data == 8'hEE) begin
                            // Query command - send just header for now (simplify)
                            if (!cmd_pending) begin
                                cmd_tx_data <= 8'hEE;
                                cmd_tx_valid <= 1;
                                cmd_pending <= 1;
                            end
                        end else if (uart_rx_data == 8'hFF) begin
                            // Debug: Force send one encoder packet immediately
                            if (!cmd_pending) begin
                                cmd_tx_data <= 8'hFF;
                                cmd_tx_valid <= 1;
                                cmd_pending <= 1;
                            end
                        end
                    end
                    
                    3'd1: begin  // High byte (or Motor ID for PID commands)
                        if (current_cmd >= 5 && current_cmd <= 9) begin
                            // PID commands: store motor ID
                            motor_id_temp <= uart_rx_data;
                            if (current_cmd == 5) begin
                                // PID Mode command: only 3 bytes, go to state 2
                                rx_state <= 2;
                            end else begin
                                // PID parameter commands: 4 bytes, go to state 2 for high byte
                                rx_state <= 2;
                            end
                        end else begin
                            // Regular commands: store high byte
                            temp_data[15:8] <= uart_rx_data;
                            rx_state <= 2;
                        end
                    end
                    
                    3'd2: begin  // Low byte (or data for PID commands)
                        if (current_cmd == 5) begin
                            // PID Mode command: 3 bytes total, execute now
                            temp_data[7:0] <= uart_rx_data;
                            rx_state <= 0;
                            
                            // Execute PID Mode command
                            if (motor_id_temp == 0) begin
                                pid_mode[0] <= uart_rx_data[0];  // Motor 0 mode
                            end else if (motor_id_temp == 1) begin
                                pid_mode[1] <= uart_rx_data[0];  // Motor 1 mode
                            end
                            if (!cmd_pending) begin
                                cmd_tx_data <= 8'h22;
                                cmd_tx_valid <= 1;
                                cmd_pending <= 1;
                            end
                        end else if (current_cmd >= 6 && current_cmd <= 9) begin
                            // PID parameter commands: 4 bytes, store high byte and continue
                            temp_data[15:8] <= uart_rx_data;
                            rx_state <= 3;
                        end else begin
                            // Regular 3-byte commands
                            temp_data[7:0] <= uart_rx_data;
                            rx_state <= 0;
                        
                            case (current_cmd)
                            4'd0: begin  // Motor 0 PWM
                                pwm_reg0 <= clamp_pwm({temp_data[15:8], uart_rx_data});
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'hAA;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd1: begin  // Motor 1 PWM
                                pwm_reg1 <= clamp_pwm({temp_data[15:8], uart_rx_data});
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'hBB;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd2: begin  // PSC
                                psc_reg <= {temp_data[15:8], uart_rx_data};
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'hCC;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd3: begin  // CCR
                                ccr_reg <= {temp_data[15:8], uart_rx_data};
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'hDD;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd4: begin  // PPR
                                ppr_reg <= {temp_data[15:8], uart_rx_data};
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h11;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd5: begin  // PID Mode
                                if (motor_id_temp == 0) begin
                                    pid_mode[0] <= uart_rx_data[0];  // Motor 0 mode
                                end else if (motor_id_temp == 1) begin
                                    pid_mode[1] <= uart_rx_data[0];  // Motor 1 mode
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h22;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd6: begin  // PID Setpoint
                                if (motor_id_temp == 0) begin
                                    pid_setpoint0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_setpoint1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h33;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd7: begin  // PID Kp
                                if (motor_id_temp == 0) begin
                                    pid_kp0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_kp1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h44;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd8: begin  // PID Ki
                                if (motor_id_temp == 0) begin
                                    pid_ki0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_ki1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h55;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd9: begin  // PID Kd
                                if (motor_id_temp == 0) begin
                                    pid_kd0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_kd1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h66;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                        endcase
                        end
                    end
                    
                    3'd3: begin  // Final byte for 4-byte PID parameter commands
                        temp_data[7:0] <= uart_rx_data;
                        rx_state <= 0;
                        
                        case (current_cmd)
                            4'd6: begin  // PID Setpoint
                                if (motor_id_temp == 0) begin
                                    pid_setpoint0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_setpoint1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h33;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd7: begin  // PID Kp
                                if (motor_id_temp == 0) begin
                                    pid_kp0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_kp1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h44;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd8: begin  // PID Ki
                                if (motor_id_temp == 0) begin
                                    pid_ki0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_ki1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h55;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                            
                            4'd9: begin  // PID Kd
                                if (motor_id_temp == 0) begin
                                    pid_kd0 <= {temp_data[15:8], uart_rx_data};
                                end else if (motor_id_temp == 1) begin
                                    pid_kd1 <= {temp_data[15:8], uart_rx_data};
                                end
                                if (!cmd_pending) begin
                                    cmd_tx_data <= 8'h66;
                                    cmd_tx_valid <= 1;
                                    cmd_pending <= 1;
                                end
                            end
                        endcase
                    end
                endcase
            end
        end
    end

    wire [1:0] dir0, dir1;

    // PID Controller outputs
    wire signed [15:0] pid_output0, pid_output1;
    wire pid_output_valid0, pid_output_valid1;
    
    // Final PWM values (either manual PWM or PID output)
    wire signed [15:0] final_pwm0, final_pwm1;
    
    // Select between manual PWM and PID output
    assign final_pwm0 = pid_mode[0] ? pid_output0 : pwm_reg0;
    assign final_pwm1 = pid_mode[1] ? pid_output1 : pwm_reg1;

    // PID Controller for Motor 0
    pid_controller u_pid0 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .setpoint(pid_setpoint0),
        .kp(pid_kp0),
        .ki(pid_ki0),
        .kd(pid_kd0),
        .feedback(enc0_rpm),
        .feedback_valid(enc0_rpm_valid),
        .pid_enable(pid_mode[0]),
        .max_output(ccr_reg),
        .pid_output(pid_output0),
        .output_valid(pid_output_valid0)
    );
    
    // PID Controller for Motor 1
    pid_controller u_pid1 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .setpoint(pid_setpoint1),
        .kp(pid_kp1),
        .ki(pid_ki1),
        .kd(pid_kd1),
        .feedback(enc1_rpm),
        .feedback_valid(enc1_rpm_valid),
        .pid_enable(pid_mode[1]),
        .max_output(ccr_reg),
        .pid_output(pid_output1),
        .output_valid(pid_output_valid1)
    );

    pwm_gen u_pwm0 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .psc(psc_reg),
        .ccr(ccr_reg),
        .pwm_in(final_pwm0),  // Use final PWM (either manual or PID)
        .pwm_out(PWM_OUT[0]),
        .motor_dir(dir0)
    );

    pwm_gen u_pwm1 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .psc(psc_reg),
        .ccr(ccr_reg),
        .pwm_in(final_pwm1),  // Use final PWM (either manual or PID)
        .pwm_out(PWM_OUT[1]),
        .motor_dir(dir1)
    );

    // Encoder position, direction and RPM
    wire signed [15:0] enc0_position, enc1_position;
    wire enc0_dir, enc1_dir;
    wire signed [15:0] enc0_rpm, enc1_rpm;
    wire enc0_rpm_valid, enc1_rpm_valid;
    
    simple_encoder u_enc0 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .enc_a(ENC_A[0]),
        .enc_b(ENC_B[0]),
        .ppr(ppr_reg),  // Use PPR value from UART
        .position(enc0_position),
        .direction(enc0_dir),
        .rpm(enc0_rpm),
        .rpm_valid(enc0_rpm_valid)
    );
    
    simple_encoder u_enc1 (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .enc_a(ENC_A[1]),
        .enc_b(ENC_B[1]),
        .ppr(ppr_reg),  // Use PPR value from UART
        .position(enc1_position),
        .direction(enc1_dir),
        .rpm(enc1_rpm),
        .rpm_valid(enc1_rpm_valid)
    );
    
    // UART TX data multiplexing with proper priority
    reg [7:0] cmd_tx_data = 0;
    reg cmd_tx_valid = 0;
    
    // Encoder data transmission (using direct RPM from encoder modules)
    wire [7:0] enc_tx_data;
    wire enc_tx_valid;
    
    encoder_data_tx u_enc_tx (
        .clk(CLK100MHZ),
        .rst_n(CPU_RESETN),
        .transmit_enable(1'b1),  // ENABLE encoder dengan rate 3Hz untuk reliability ultimate
        .enc0_pos(enc0_position),
        .enc1_pos(enc1_position),
        .rpm0(enc0_rpm),         
        .rpm1(enc1_rpm),         
        .enc0_dir(enc0_dir),     
        .enc1_dir(enc1_dir),
        .uart_tx_busy(uart_tx_busy | cmd_pending | cmd_transmission_active | command_window_active | cmd_needs_retry),  // Block encoder during any command activity
        .uart_tx_data(enc_tx_data),
        .uart_tx_valid(enc_tx_valid)
    );
    
    // UART TX multiplexer - Command has ABSOLUTE priority with command queue
    reg [15:0] cmd_delay = 0;
    reg cmd_transmission_active = 0;
    reg command_window_active = 0;  // Extended blocking window for commands
    reg [2:0] cmd_retry_count = 0;   // Retry mechanism for failed commands
    reg cmd_needs_retry = 0;
    
    always @(posedge CLK100MHZ or negedge CPU_RESETN) begin
        if (!CPU_RESETN) begin
            uart_tx_data <= 0;
            uart_tx_valid <= 0;
            cmd_delay <= 0;
            cmd_transmission_active <= 0;
            command_window_active <= 0;
            cmd_retry_count <= 0;
            cmd_needs_retry <= 0;
        end else begin
            uart_tx_valid <= 0;
            
            // Decrement delay counter
            if (cmd_delay > 0) begin
                cmd_delay <= cmd_delay - 1;
                command_window_active <= 1;  // Keep command window active during delay
            end else begin
                command_window_active <= 0;
                // Clear retry flag when delay expires
                if (cmd_needs_retry && cmd_retry_count < 3) begin
                    cmd_needs_retry <= 0;
                    cmd_retry_count <= cmd_retry_count + 1;
                end else begin
                    cmd_retry_count <= 0;
                end
            end
            
            // Command transmission state machine with retry mechanism
            if ((cmd_tx_valid || cmd_needs_retry) && !cmd_transmission_active && !command_window_active) begin
                // Start command transmission immediately - ensure UART is completely free
                if (!uart_tx_busy) begin
                    uart_tx_data <= cmd_tx_data;
                    uart_tx_valid <= 1;
                    cmd_transmission_active <= 1;
                    cmd_delay <= 15000;  // 150μs delay after command (15000 clocks at 100MHz) - increased
                end else begin
                    // UART still busy, set retry flag
                    cmd_needs_retry <= 1;
                end
            end else if (cmd_transmission_active && !uart_tx_busy) begin
                // Command transmission completed
                cmd_transmission_active <= 0;
            end
            // Encoder data transmission - only when absolutely no command activity
            else if (enc_tx_valid && !uart_tx_busy && !cmd_pending && !cmd_transmission_active && !command_window_active && !cmd_needs_retry) begin
                uart_tx_data <= enc_tx_data;
                uart_tx_valid <= 1;
            end
        end
    end

    assign MOTOR_IN = {dir1, dir0};
    // Debug LED: Show retry state, encoder activity, and encoder positions
    // assign LED = {cmd_needs_retry, enc_tx_valid, |enc0_position[15:8], |enc1_position[15:8]};
    assign LED = {dir1, dir0};

endmodule