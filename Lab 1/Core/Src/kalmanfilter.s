
.syntax unified

.global kalmanfilter


kalmanfilter:
	PUSH {R4, R5}
	VLDR.f32 S0, [R1] //measurement
	VLDR.f32 S1,[R0] //self.q. Also address of struct
	VLDR.f32 S2, [R0, #4] //self.r
	VLDR.f32 S3, [R0, #8] //self.x
	VLDR.f32 S4, [R0, #12] //self.p
	VLDR.f32 S5, [R0, #16] //self.k

update:
	VADD.f32 S4, S4, S1 //self.p = self.p + self.q
	VADD.f32 S6, S4, S2 //self.p + self.r
	VDIV.f32 S5, S4, S6 //slef.k = self.p /(self.p + self.r)
	VSUB.f32 S7, S0, S3 //measurement - self.x
	VMUL.f32 S8, S5, S7 //self.k * (measurement - self.x)
	VADD.f32 S3, S3, S8 //self.x = self.x + self.k * (measurement - self.x)
	VMUL.f32 S9, S4, S5 //self.p * self.k
	VSUB.f32 S4, S4, S9 //self.p = self.p - (self.p * self.k)



done:
	VSTR.f32 S1, [R0]
	VSTR.f32 S2, [R0, #4]
	VSTR.f32 S3, [R0, #8]
	VSTR.f32 S4, [R0, #12]
	VSTR.f32 S5, [R0, #16]
	POP {R4, R5}
	BX LR
