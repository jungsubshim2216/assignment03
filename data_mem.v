/* DO NOT MODIFY THIS MODULE */
module data_mem(
    output [31:0] ReadData,
    input  [31:0] ADDR, WriteData,
    input  clk, MemWrite,
    input  pc_en
);

reg [31:0] D_Memory [0:31];

wire [29:0] mem_offset;

assign mem_offset = ADDR[31:2];

assign ReadData = D_Memory[mem_offset];


always @ (posedge clk) begin
    if(pc_en) begin
        if (MemWrite) begin
            D_Memory[mem_offset] <= WriteData;
        end
    end
end

initial begin
  D_Memory[0] = 32'd6;
  D_Memory[1] = 32'd10;
  D_Memory[2] = 32'd2;
  D_Memory[3] = 32'd23;
  D_Memory[4] = 32'd34;
  D_Memory[5] = 32'd4;
  D_Memory[6] = 32'd9;
  D_Memory[7] = 32'd74;
  D_Memory[8] = 32'd9;
  D_Memory[9] = 32'd38;
  D_Memory[10] = 32'd19;
  D_Memory[11] = 32'd3;
  D_Memory[12] = 32'd5;
  D_Memory[13] = 32'd62;
  D_Memory[14] = 32'd11;
  D_Memory[15] = 32'd1;
end

endmodule