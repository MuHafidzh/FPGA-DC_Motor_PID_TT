module pwm_gen (
    input wire clk,
    input wire rst_n,
    input wire [15:0] psc,      // prescaler (dynamic)
    input wire [15:0] ccr,      // counter top (dynamic)
    input wire signed [15:0] pwm_in, // signed PWM input
    output reg pwm_out,
    output reg [1:0] motor_dir // [1]=IN1, [0]=IN2
);

    reg [15:0] prescaler = 0;
    reg [15:0] counter = 0;
    reg [15:0] pwm_val = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prescaler <= 0;
            counter <= 0;
            pwm_out <= 0;
            pwm_val <= 0;
            motor_dir <= 2'b00;
        end else begin
            // Prescaler (dynamic)
            if (prescaler >= psc) begin
                prescaler <= 0;
                // Counter (dynamic)
                if (counter >= ccr)
                    counter <= 0;
                else
                    counter <= counter + 1;
            end else begin
                prescaler <= prescaler + 1;
            end

            // Direction logic
            if (pwm_in > 0) begin
                pwm_val <= (pwm_in > ccr) ? ccr : pwm_in;
                motor_dir <= 2'b10; // Forward: IN1=1, IN2=0
            end else if (pwm_in < 0) begin
                pwm_val <= ((-pwm_in) > ccr) ? ccr : -pwm_in;
                motor_dir <= 2'b01; // Reverse: IN1=0, IN2=1
            end else begin
                pwm_val <= 0;
                motor_dir <= 2'b00; // Stop
            end

            // PWM output
            pwm_out <= (counter < pwm_val) ? 1'b1 : 1'b0;
        end
    end

endmodule