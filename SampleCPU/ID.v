`include "lib/defines.vh"
module ID(
    input wire clk,
    input wire rst,
    // input wire flush,
    input wire [`StallBus-1:0] stall,
    
    output wire stallreq,

    input wire [`IF_TO_ID_WD-1:0] if_to_id_bus,

    input wire [31:0] inst_sram_rdata,

    input wire [`WB_TO_RF_WD-1:0] wb_to_rf_bus,
    input wire [`EX_TO_RF_WD-1:0] ex_to_rf_bus,
    input wire [`MEM_TO_RF_WD-1:0] mem_to_rf_bus,

    output wire [`ID_TO_EX_WD-1:0] id_to_ex_bus,

    output wire [`BR_WD-1:0] br_bus 
);

    reg [`IF_TO_ID_WD-1:0] if_to_id_bus_r;
    wire [31:0] inst;
    wire [31:0] id_pc;
    wire ce;

    wire wb_rf_we, ex_rf_we, mem_rf_we;
    wire [4:0] wb_rf_waddr, ex_rf_waddr, mem_rf_waddr;
    wire [31:0] wb_rf_wdata, ex_rf_wdata, mem_rf_wdata;

    always @ (posedge clk) begin
        if (rst) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;        
        end
        // else if (flush) begin
        //     ic_to_id_bus <= `IC_TO_ID_WD'b0;
        // end
        else if (stall[1]==`Stop && stall[2]==`NoStop) begin
            if_to_id_bus_r <= `IF_TO_ID_WD'b0;
        end
        else if (stall[1]==`NoStop) begin
            if_to_id_bus_r <= if_to_id_bus;
        end
    end
    
    assign inst = inst_sram_rdata;
    assign {
        ce,
        id_pc
    } = if_to_id_bus_r;

    assign {
        ex_rf_we,
        ex_rf_waddr,
        ex_rf_wdata
    } = ex_to_rf_bus;
    assign {
        mem_rf_we,
        mem_rf_waddr,
        mem_rf_wdata
    } = mem_to_rf_bus;
    assign {
        wb_rf_we,
        wb_rf_waddr,
        wb_rf_wdata
    } = wb_to_rf_bus;

    wire [5:0] opcode;
    wire [4:0] rs,rt,rd,sa;
    wire [5:0] func;
    wire [15:0] imm;
    wire [25:0] instr_index;
    wire [19:0] code;
    wire [4:0] base;
    wire [15:0] offset;
    wire [2:0] sel;

    wire [63:0] op_d, func_d;
    wire [31:0] rs_d, rt_d, rd_d, sa_d;

    wire [2:0] sel_alu_src1;
    wire [3:0] sel_alu_src2;
    wire [11:0] alu_op;

    wire [1:0] mem_op;
    wire data_ram_en;
    wire [3:0] data_ram_wen;
    
    wire rf_we;
    wire [4:0] rf_waddr;
    wire sel_rf_res;
    wire [2:0] sel_rf_dst;

    wire [31:0] rdata1, rdata2, rf_rdata1, rf_rdata2;
    

    regfile u_regfile(
    	.clk    (clk    ),
        .raddr1 (rs ),
        .rdata1 (rf_rdata1 ),
        .raddr2 (rt ),
        .rdata2 (rf_rdata2 ),
        .we     (wb_rf_we     ),
        .waddr  (wb_rf_waddr  ),
        .wdata  (wb_rf_wdata  )
    );

    assign rdata1 = {ex_rf_we & {ex_rf_waddr == rs}} ? ex_rf_wdata : 
                    {mem_rf_we & {mem_rf_waddr == rs}} ? mem_rf_wdata :
                    {wb_rf_we & {wb_rf_waddr == rs}} ? wb_rf_wdata :
                                                        rf_rdata1;
    


    assign rdata2 = {ex_rf_we & {ex_rf_waddr == rt}} ? ex_rf_wdata : 
                    {mem_rf_we & {mem_rf_waddr == rt}} ? mem_rf_wdata :
                    {wb_rf_we & {wb_rf_waddr == rt}} ? wb_rf_wdata :
                                                        rf_rdata2;


    assign opcode = inst[31:26];
    assign rs = inst[25:21];
    assign rt = inst[20:16];
    assign rd = inst[15:11];
    assign sa = inst[10:6];
    assign func = inst[5:0];
    assign imm = inst[15:0];
    assign instr_index = inst[25:0];
    assign code = inst[25:6];
    assign base = inst[25:21];
    assign offset = inst[15:0];
    assign sel = inst[2:0];

    wire inst_ori, inst_or, inst_xor; 
    wire inst_lui, inst_beq, inst_subu;
    wire inst_addu, inst_addiu;
    wire inst_jr, inst_jal;
    wire inst_bne;
    wire inst_sll;
    wire inst_sw;
    wire inst_lw;




    wire op_add, op_sub, op_slt, op_sltu;
    wire op_and, op_nor, op_or, op_xor;
    wire op_sll, op_srl, op_sra, op_lui;

    decoder_6_64 u0_decoder_6_64(
    	.in  (opcode  ),
        .out (op_d )
    );

    decoder_6_64 u1_decoder_6_64(
    	.in  (func  ),
        .out (func_d )
    );
    
    decoder_5_32 u0_decoder_5_32(
    	.in  (rs  ),
        .out (rs_d )
    );

    decoder_5_32 u1_decoder_5_32(
    	.in  (rt  ),
        .out (rt_d )
    );

    decoder_5_32 u2_decoder_5_32(
    	.in  (sa  ),
        .out (sa_d )
    );

    decoder_5_32 u3_decoder_5_32(
    	.in  (rd  ),
        .out (rd_d )
    );

    
    assign inst_ori     = op_d[6'b00_1101];
    assign inst_lui     = op_d[6'b00_1111];
    assign inst_addiu   = op_d[6'b00_1001];
    assign inst_beq     = op_d[6'b00_0100];
    assign inst_subu    = op_d[6'b00_0000] & sa_d[5'b0_0000] & func_d[6'b10_0011];
    assign inst_jr      = op_d[6'b00_0000] & rt_d[5'b0_0000] & rd_d[5'b0_0000] & sa_d[5'b0_0000] & func_d[6'b00_1000];
    assign inst_jal     = op_d[6'b00_0011];
    assign inst_addu    = op_d[6'b00_0000] & sa_d[5'b0_0000] & func_d[6'b10_0001];
    assign inst_bne     = op_d[6'b00_0101];
    assign inst_sll     = op_d[6'b00_0000] & rs_d[5'b0_0000] & func_d[6'b00_0000];
    assign inst_or      = op_d[6'b00_0000] & sa_d[5'b0_0000] & func_d[6'b10_0101];
    assign inst_sw      = op_d[6'b10_1011];
    assign inst_lw      = op_d[6'b10_0011];
    assign inst_xor     = op_d[6'b00_0000] & sa_d[5'b0_0000] & func_d[6'b10_0110];

    // rs to reg1
    assign sel_alu_src1[0] = inst_ori | inst_addiu | inst_addu | inst_subu | inst_or | inst_sw 
                            | inst_lw | inst_xor;

    // pc to reg1
    assign sel_alu_src1[1] = inst_jal;

    // sa_zero_extend to reg1
    assign sel_alu_src1[2] = inst_sll;

    
    // rt to reg2
    assign sel_alu_src2[0] = inst_subu | inst_addu | inst_sll | inst_or | inst_xor;
    
    // imm_sign_extend to reg2
    assign sel_alu_src2[1] = inst_lui | inst_addiu | inst_sw | inst_lw;

    // 32'b8 to reg2
    assign sel_alu_src2[2] = inst_jal;

    // imm_zero_extend to reg2
    assign sel_alu_src2[3] = inst_ori;



    assign op_add = inst_addiu | inst_addu | inst_jal | inst_sw | inst_lw;
    assign op_sub = inst_subu;
    assign op_slt = 1'b0;
    assign op_sltu = 1'b0;
    assign op_and = 1'b0;
    assign op_nor = 1'b0;
    assign op_or = inst_ori | inst_or;
    assign op_xor = inst_xor;
    assign op_sll = inst_sll;
    assign op_srl = 1'b0;
    assign op_sra = 1'b0;
    assign op_lui = inst_lui;

    assign alu_op = {op_add, op_sub, op_slt, op_sltu,
                     op_and, op_nor, op_or, op_xor,
                     op_sll, op_srl, op_sra, op_lui};

    assign mem_op = {inst_lw, inst_sw};

    // load and store enable
    assign data_ram_en = inst_sw | inst_lw;

    // write enable
    assign data_ram_wen = inst_sw;

    // 0 from alu_res ; 1 from ld_res
    assign sel_rf_res = inst_lw; 

    // regfile store enable
    assign rf_we = inst_ori | inst_lui | inst_addiu | inst_subu | inst_jal | inst_addu | inst_sll | inst_or
                    | inst_lw | inst_xor;



    // store in [rd]
    assign sel_rf_dst[0] = inst_subu | inst_addu | inst_sll | inst_or | inst_xor;
    // store in [rt] 
    assign sel_rf_dst[1] = inst_ori | inst_lui | inst_addiu | inst_lw;
    // store in [31]
    assign sel_rf_dst[2] = inst_jal;

    // sel for regfile address
    assign rf_waddr = {5{sel_rf_dst[0]}} & rd 
                    | {5{sel_rf_dst[1]}} & rt
                    | {5{sel_rf_dst[2]}} & 32'd31;



    assign id_to_ex_bus = {
        mem_op,         // 160:159
        id_pc,          // 158:127
        inst,           // 126:95
        alu_op,         // 94:83
        sel_alu_src1,   // 82:80
        sel_alu_src2,   // 79:76
        data_ram_en,    // 75
        data_ram_wen,   // 74:71
        rf_we,          // 70
        rf_waddr,       // 69:65
        sel_rf_res,     // 64
        rdata1,         // 63:32
        rdata2          // 31:0
    };


    wire br_e;
    wire [31:0] br_addr;
    wire rs_eq_rt;
    wire rs_ge_z;
    wire rs_gt_z;
    wire rs_le_z;
    wire rs_lt_z;
    wire [31:0] pc_plus_4;
    assign pc_plus_4 = id_pc + 32'h4;

    assign rs_eq_rt = (rdata1 == rdata2);

    assign br_e = inst_beq & rs_eq_rt
                | inst_bne & ~rs_eq_rt
                | inst_jr
                | inst_jal;
    assign br_addr = inst_beq ? (pc_plus_4 + {{14{inst[15]}},inst[15:0],2'b0}) 
                    : inst_bne ? (pc_plus_4 + {{14{inst[15]}},inst[15:0],2'b0}) 
                    : inst_jr ? rdata1
                    : inst_jal ? {id_pc[31:28],instr_index,2'b0}
                    : 32'b0;

    assign br_bus = {
        br_e,
        br_addr
    };
    


endmodule