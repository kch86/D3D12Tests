//=================================================================================================
//
//  DXR Path Tracer
//  by MJP
//  http://mynameismjp.wordpress.com/
//
//  All code and content licensed under the MIT license
//
//=================================================================================================

// ================================================================================================
// Constant buffers
// ================================================================================================
struct VSConstants
{
	row_major float4x4 World;
	row_major float4x4 View;
	row_major float4x4 WorldViewProjection;
	float NearClip;
	float FarClip;
};

ConstantBuffer<VSConstants> VSCBuffer : register(b0);

// ================================================================================================
// Input/Output structs
// ================================================================================================
//struct VSInput
//{
//    float4 PositionOS 		: POSITION;
//};

struct VSInput
{
	float3 PositionOS 		    : POSITION;
	float3 NormalOS 		    : NORMAL;
	float2 UV 		            : UV;
	float3 TangentOS 		    : TANGENT;
	float3 BitangentOS		    : BITANGENT;
};

struct VSOutput
{
    float4 PositionCS 		: SV_Position;
};

// ================================================================================================
// Vertex Shader
// ================================================================================================
VSOutput VS(in VSInput input)
{
    VSOutput output;

    // Calc the clip-space position
    output.PositionCS = mul(float4(input.PositionOS, 1.0f), VSCBuffer.WorldViewProjection);

    return output;
}

float PS(in VSOutput input) : SV_TARGET
{
	const float Max_Filer_Kernel_Size = 7.0;
	float z = input.PositionCS.z / input.PositionCS.w;
	z += 1.0f / 65535.0;
	z += (abs(ddx(z)) + abs(ddy(z))) * Max_Filer_Kernel_Size;
	return z;
}