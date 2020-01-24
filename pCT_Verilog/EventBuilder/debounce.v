//////////////////////////////////////////////////////////////////////////////////
// Create Date:    16:49:06 09/24/2012 
// Design Name: 
// Module Name:    debounce 
// David Steinberg
// Debound the push-button switch used to reset the event builder
//////////////////////////////////////////////////////////////////////////////////
module debounce(clock, buttonPress, pulse, reset
    );
    input clock, buttonPress, reset;
	 output pulse;

   reg [2:0] regs;

   always @ (posedge clock)
      if (reset == 1)
         regs <= 3'b000;
      else
         regs <= {regs[1:0], buttonPress};

   assign pulse = regs[0] & regs[1] & !regs[2];

endmodule
