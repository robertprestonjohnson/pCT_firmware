// Queue the buffer ready signals and trigger tags from the tracker front-end boards
// R. Johnson,  September 8, 2013
module TagFIFO(overFlow, dataOut, notEmpty, enableOut, dataIn, enableIn, Clock, Reset, Board);
input [2:0] Board;
input Clock;
input Reset;
input enableIn;
input [1:0] dataIn;
input enableOut;
output notEmpty;
output [1:0] dataOut;
output overFlow;

reg overFlow;
reg [1:0] dataOut;
reg [3:0] nStored, writePtr, readPtr;
reg [1:0] Tags [0:15];
wire notEmpty;

assign notEmpty = (nStored != 0);

always @ (posedge Clock) begin
	if (Reset) begin
		overFlow <= 1'b0;
		writePtr <= 0;
		readPtr <= 0;
		nStored <= 0;
	end else begin
		if (enableIn) begin
			if (!enableOut) nStored <= nStored + 1;
			if (nStored == 15) overFlow <= 1'b1;
			else overFlow <= 1'b0;
			writePtr <= writePtr + 1;
			Tags[writePtr] <= dataIn;
			$display("%g\t TagFIFO %d: queuing tag %d in location %d, nStored=%d.",$time,Board,dataIn,writePtr,nStored);
		end
		if (enableOut & notEmpty) begin
			if (!enableIn) nStored <= nStored - 1;
			readPtr <= readPtr + 1;
			dataOut <= Tags[readPtr];
			$display("%g\t TagFIFO %d: outputing tag %d, readPtr=%d, nStored=%d.",$time,Board,Tags[readPtr],readPtr,nStored);
		end
	end
end


endmodule
