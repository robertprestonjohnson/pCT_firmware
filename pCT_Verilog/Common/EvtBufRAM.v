//FPGA RAM used by the event builder to buffer tracker and energy data
//Dual ported RAM with one write port
//R. Johnson
module EvtBufRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [7:0] addra, addrb;   //Addresses for the primary and secondary ports
input [11:0] dia;           //Input data register to write to memory
output [11:0] doa, dob;     //Output registers for the two ports
reg [11:0] RAM [255:0];     //Memory array
reg [11:0] dob, doa;

always @(posedge clka)
begin
	if (ena)
	begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
//			$display("%g\t EvtBufRAM: storing %b in address %d",$time,dia,addra);
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb)
	begin
		dob <= RAM[addrb];
//		$display("%g\t EvtBufRAM: reading %b from address %d",$time,RAM[addrb],addrb);
	end
end
endmodule