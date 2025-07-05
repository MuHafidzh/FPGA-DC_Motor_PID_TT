`timescale 1ns / 1ps
`default_nettype none

module tb ();

    //dumpvar vcd
    initial begin
      $dumpfile("tb.vcd");
      $dumpvars(0, tb);
      #1;
    end

    // Clock and reset
    reg clk;
    reg CPU_RESETN;

    // UART
    reg UART_RX;
    wire UART_TX;

    // PWM outputs
    wire [1:0] PWM_OUT;

    // Motor direction signals
    wire [3:0] MOTOR_IN;

    // LEDs
    wire [3:0] LED;

    // Encoder inputs
    reg [1:0] ENC_A;
    reg [1:0] ENC_B;

    // Instantiate the top module
    tt_um_top_motor_control uut (
        .clk(clk),
        .CPU_RESETN(CPU_RESETN),
        .UART_RX(UART_RX),
        .UART_TX(UART_TX),
        .PWM_OUT(PWM_OUT),
        .MOTOR_IN(MOTOR_IN),
        .LED(LED),
        .ENC_A(ENC_A),
        .ENC_B(ENC_B)
    );

endmodule