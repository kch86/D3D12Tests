//=================================================================================================
//
//  DXR Path Tracer
//  by MJP
//  http://mynameismjp.wordpress.com/
//
//  All code and content licensed under the MIT license
//
//=================================================================================================

#include <PCH.h>

#include "MeshRenderer.h"

#include <Exceptions.h>
#include <Utility.h>
#include <Graphics/ShaderCompilation.h>
#include <Graphics/Skybox.h>
#include <Graphics/Profiler.h>

#include "AppSettings.h"

// Constants
static const uint64 SunShadowMapSize = 2048;
static const uint64 SpotLightShadowMapSize = 1024;

enum MainPassRootParams
{
    MainPass_StandardDescriptors,
    MainPass_VSCBuffer,
    MainPass_PSCBuffer,
    MainPass_ShadowCBuffer,
    MainPass_MatIndexCBuffer,
    MainPass_LightCBuffer,
    MainPass_SRVIndices,
    MainPass_AppSettings,

    NumMainPassRootParams,
};

struct MeshVSConstants
{
    Float4Align Float4x4 World;
    Float4Align Float4x4 View;
    Float4Align Float4x4 WorldViewProjection;
    float NearClip = 0.0f;
    float FarClip = 0.0f;
};

// Frustum culls meshes, and produces a buffer of visible mesh indices
static uint64 CullMeshes(const Camera& camera, const Array<DirectX::BoundingBox>& boundingBoxes, Array<uint32>& drawIndices)
{
    DirectX::BoundingFrustum frustum(camera.ProjectionMatrix().ToSIMD());
    frustum.Transform(frustum, 1.0f, camera.Orientation().ToSIMD(), camera.Position().ToSIMD());

    uint64 numVisible = 0;
    const uint64 numMeshes = boundingBoxes.Size();
    for(uint64 i = 0; i < numMeshes; ++i)
    {
        if(frustum.Intersects(boundingBoxes[i]))
            drawIndices[numVisible++] = uint32(i);
    }

    return numVisible;
}

struct OBBIntersectData
{
	DirectX::XMVECTOR RX0;
	DirectX::XMVECTOR RX1;
	DirectX::XMVECTOR RX2;
	DirectX::XMVECTOR R0X;
	DirectX::XMVECTOR R1X;
	DirectX::XMVECTOR R2X;
	DirectX::XMVECTOR AR0X;
	DirectX::XMVECTOR AR1X;
	DirectX::XMVECTOR AR2X;
	DirectX::XMVECTOR ARX0;
	DirectX::XMVECTOR ARX1;
	DirectX::XMVECTOR ARX2;

};

static void OBBIntersectPrologue(OBBIntersectData& obi, const DirectX::BoundingOrientedBox& obb)
{
	using namespace DirectX;

	XMVECTOR A_quat = XMLoadFloat4(&obb.Orientation);

	XMMATRIX R = XMMatrixRotationQuaternion(A_quat);

	// Rows. Note R[0,1,2]X.w = 0.
	obi.R0X = R.r[0];
	obi.R1X = R.r[1];
	obi.R2X = R.r[2];

	R = XMMatrixTranspose(R);

	// Columns. Note RX[0,1,2].w = 0.
	obi.RX0 = R.r[0];
	obi.RX1 = R.r[1];
	obi.RX2 = R.r[2];

	// Absolute value of rows.
	obi.AR0X = XMVectorAbs(obi.R0X);
	obi.AR1X = XMVectorAbs(obi.R1X);
	obi.AR2X = XMVectorAbs(obi.R2X);

	// Absolute value of columns.
	obi.ARX0 = XMVectorAbs(obi.RX0);
	obi.ARX1 = XMVectorAbs(obi.RX1);
	obi.ARX2 = XMVectorAbs(obi.RX2);
}

__forceinline static bool OBBIntersect(const OBBIntersectData& obi, const DirectX::BoundingOrientedBox& a, const DirectX::BoundingBox& _b)
{
	using namespace DirectX;
	BoundingOrientedBox b(_b.Center, _b.Extents, XMFLOAT4(0.f, 0.f, 0.f, 1.f));

	// Build the 3x3 rotation matrix that defines the orientation of B relative to A.
	XMVECTOR A_quat = XMLoadFloat4(&a.Orientation);
	XMVECTOR B_quat = XMLoadFloat4(&b.Orientation);

	//assert(DirectX::Internal::XMQuaternionIsUnit(A_quat));
	//assert(DirectX::Internal::XMQuaternionIsUnit(B_quat));

	XMVECTOR Q = A_quat;// XMQuaternionMultiply(A_quat, XMQuaternionConjugate(B_quat));
	XMMATRIX R = XMMatrixRotationQuaternion(Q);

	// Compute the translation of B relative to A.
	XMVECTOR A_cent = XMLoadFloat3(&a.Center);
	XMVECTOR B_cent = XMLoadFloat3(&b.Center);
	XMVECTOR t = XMVector3InverseRotate(XMVectorSubtract(B_cent, A_cent), A_quat);

	//
	// h(A) = extents of A.
	// h(B) = extents of B.
	//
	// a(u) = axes of A = (1,0,0), (0,1,0), (0,0,1)
	// b(u) = axes of B relative to A = (r00,r10,r20), (r01,r11,r21), (r02,r12,r22)
	//  
	// For each possible separating axis l:
	//   d(A) = sum (for i = u,v,w) h(A)(i) * abs( a(i) dot l )
	//   d(B) = sum (for i = u,v,w) h(B)(i) * abs( b(i) dot l )
	//   if abs( t dot l ) > d(A) + d(B) then disjoint
	//

	// Load extents of A and B.
	XMVECTOR h_A = XMLoadFloat3(&a.Extents);
	XMVECTOR h_B = XMLoadFloat3(&b.Extents);

	// Rows. Note R[0,1,2]X.w = 0.
	//XMVECTOR obi.R0X = R.r[0];
	//XMVECTOR obi.obi.R1X = R.r[1];
	//XMVECTOR obi.R2X = R.r[2];

	//R = XMMatrixTranspose(R);

	//// Columns. Note RX[0,1,2].w = 0.
	//XMVECTOR obi.RX0 = R.r[0];
	//XMVECTOR obi.RX1 = R.r[1];
	//XMVECTOR obi.RX2 = R.r[2];

	//// Absolute value of rows.
	//XMVECTOR AR0X = XMVectorAbs(obi.R0X);
	//XMVECTOR AR1X = XMVectorAbs(obi.R1X);
	//XMVECTOR AR2X = XMVectorAbs(obi.R2X);

	//// Absolute value of columns.
	//XMVECTOR ARX0 = XMVectorAbs(obi.RX0);
	//XMVECTOR ARX1 = XMVectorAbs(obi.RX1);
	//XMVECTOR ARX2 = XMVectorAbs(obi.RX2);

	// Test each of the 15 possible seperating axii.
	XMVECTOR d, d_A, d_B;

	// l = a(u) = (1, 0, 0)
	// t dot l = t.x
	// d(A) = h(A).x
	// d(B) = h(B) dot abs(r00, r01, r02)
	d = XMVectorSplatX(t);
	d_A = XMVectorSplatX(h_A);
	d_B = XMVector3Dot(h_B, obi.AR0X);
	XMVECTOR NoIntersection = XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B));

	// l = a(v) = (0, 1, 0)
	// t dot l = t.y
	// d(A) = h(A).y
	// d(B) = h(B) dot abs(r10, r11, r12)
	d = XMVectorSplatY(t);
	d_A = XMVectorSplatY(h_A);
	d_B = XMVector3Dot(h_B, obi.AR1X);
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(w) = (0, 0, 1)
	// t dot l = t.z
	// d(A) = h(A).z
	// d(B) = h(B) dot abs(r20, r21, r22)
	d = XMVectorSplatZ(t);
	d_A = XMVectorSplatZ(h_A);
	d_B = XMVector3Dot(h_B, obi.AR2X);
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = b(u) = (r00, r10, r20)
	// d(A) = h(A) dot abs(r00, r10, r20)
	// d(B) = h(B).x
	d = XMVector3Dot(t, obi.RX0);
	d_A = XMVector3Dot(h_A, obi.ARX0);
	d_B = XMVectorSplatX(h_B);
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = b(v) = (r01, r11, r21)
	// d(A) = h(A) dot abs(r01, r11, r21)
	// d(B) = h(B).y
	d = XMVector3Dot(t, obi.RX1);
	d_A = XMVector3Dot(h_A, obi.ARX1);
	d_B = XMVectorSplatY(h_B);
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = b(w) = (r02, r12, r22)
	// d(A) = h(A) dot abs(r02, r12, r22)
	// d(B) = h(B).z
	d = XMVector3Dot(t, obi.RX2);
	d_A = XMVector3Dot(h_A, obi.ARX2);
	d_B = XMVectorSplatZ(h_B);
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(u) x b(u) = (0, -r20, r10)
	// d(A) = h(A) dot abs(0, r20, r10)
	// d(B) = h(B) dot abs(0, r02, r01)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0W, XM_PERMUTE_1Z, XM_PERMUTE_0Y, XM_PERMUTE_0X>(obi.RX0, XMVectorNegate(obi.RX0)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.ARX0));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.AR0X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(u) x b(v) = (0, -r21, r11)
	// d(A) = h(A) dot abs(0, r21, r11)
	// d(B) = h(B) dot abs(r02, 0, r00)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0W, XM_PERMUTE_1Z, XM_PERMUTE_0Y, XM_PERMUTE_0X>(obi.RX1, XMVectorNegate(obi.RX1)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.ARX1));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.AR0X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(u) x b(w) = (0, -r22, r12)
	// d(A) = h(A) dot abs(0, r22, r12)
	// d(B) = h(B) dot abs(r01, r00, 0)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0W, XM_PERMUTE_1Z, XM_PERMUTE_0Y, XM_PERMUTE_0X>(obi.RX2, XMVectorNegate(obi.RX2)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.ARX2));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.AR0X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(v) x b(u) = (r20, 0, -r00)
	// d(A) = h(A) dot abs(r20, 0, r00)
	// d(B) = h(B) dot abs(0, r12, r11)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0Z, XM_PERMUTE_0W, XM_PERMUTE_1X, XM_PERMUTE_0Y>(obi.RX0, XMVectorNegate(obi.RX0)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.ARX0));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.AR1X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(v) x b(v) = (r21, 0, -r01)
	// d(A) = h(A) dot abs(r21, 0, r01)
	// d(B) = h(B) dot abs(r12, 0, r10)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0Z, XM_PERMUTE_0W, XM_PERMUTE_1X, XM_PERMUTE_0Y>(obi.RX1, XMVectorNegate(obi.RX1)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.ARX1));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.AR1X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(v) x b(w) = (r22, 0, -r02)
	// d(A) = h(A) dot abs(r22, 0, r02)
	// d(B) = h(B) dot abs(r11, r10, 0)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_0Z, XM_PERMUTE_0W, XM_PERMUTE_1X, XM_PERMUTE_0Y>(obi.RX2, XMVectorNegate(obi.RX2)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.ARX2));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.AR1X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(w) x b(u) = (-r10, r00, 0)
	// d(A) = h(A) dot abs(r10, r00, 0)
	// d(B) = h(B) dot abs(0, r22, r21)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_1Y, XM_PERMUTE_0X, XM_PERMUTE_0W, XM_PERMUTE_0Z>(obi.RX0, XMVectorNegate(obi.RX0)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.ARX0));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_W, XM_SWIZZLE_Z, XM_SWIZZLE_Y, XM_SWIZZLE_X>(obi.AR2X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(w) x b(v) = (-r11, r01, 0)
	// d(A) = h(A) dot abs(r11, r01, 0)
	// d(B) = h(B) dot abs(r22, 0, r20)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_1Y, XM_PERMUTE_0X, XM_PERMUTE_0W, XM_PERMUTE_0Z>(obi.RX1, XMVectorNegate(obi.RX1)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.ARX1));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Z, XM_SWIZZLE_W, XM_SWIZZLE_X, XM_SWIZZLE_Y>(obi.AR2X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// l = a(w) x b(w) = (-r12, r02, 0)
	// d(A) = h(A) dot abs(r12, r02, 0)
	// d(B) = h(B) dot abs(r21, r20, 0)
	d = XMVector3Dot(t, XMVectorPermute<XM_PERMUTE_1Y, XM_PERMUTE_0X, XM_PERMUTE_0W, XM_PERMUTE_0Z>(obi.RX2, XMVectorNegate(obi.RX2)));
	d_A = XMVector3Dot(h_A, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.ARX2));
	d_B = XMVector3Dot(h_B, XMVectorSwizzle<XM_SWIZZLE_Y, XM_SWIZZLE_X, XM_SWIZZLE_W, XM_SWIZZLE_Z>(obi.AR2X));
	NoIntersection = XMVectorOrInt(NoIntersection,
		XMVectorGreater(XMVectorAbs(d), XMVectorAdd(d_A, d_B)));

	// No seperating axis found, boxes must intersect.
	return XMVector4NotEqualInt(NoIntersection, XMVectorTrueInt()) ? true : false;
}

// Frustum culls meshes for an orthographic projection, and produces a buffer of visible mesh indices
static uint64 CullMeshesOrthographic(const OrthographicCamera& camera, bool ignoreNearZ, const Array<DirectX::BoundingBox>& boundingBoxes, Array<uint32>& drawIndices)
{
    Float3 mins = Float3(camera.MinX(), camera.MinY(), camera.NearClip());
    Float3 maxes = Float3(camera.MaxX(), camera.MaxY(), camera.FarClip());
    if(ignoreNearZ)
        mins.z = -10000.0f;

    Float3 extents = (maxes - mins) / 2.0f;
    Float3 center = mins + extents;
    center = Float3::Transform(center, camera.Orientation());
    center += camera.Position();

    DirectX::BoundingOrientedBox obb;
    obb.Extents = extents.ToXMFLOAT3();
    obb.Center = center.ToXMFLOAT3();
    obb.Orientation = camera.Orientation().ToXMFLOAT4();
	
	OBBIntersectData obiData;
	OBBIntersectPrologue(obiData, obb);

    uint64 numVisible = 0;
    const uint64 numMeshes = boundingBoxes.Size();
    for(uint64 i = 0; i < numMeshes; ++i)
    {
        //if(obb.Intersects(boundingBoxes[i]))
		if(OBBIntersect(obiData, obb, boundingBoxes[i]))
            drawIndices[numVisible++] = uint32(i);
    }

    return numVisible;
}

MeshRenderer::MeshRenderer()
{
}

void MeshRenderer::LoadShaders()
{
    // Load the mesh shaders
    meshDepthVS = CompileFromFile(L"DepthOnly.hlsl", "VS", ShaderType::Vertex);
	meshDepthPS = CompileFromFile(L"DepthOnly.hlsl", "PS", ShaderType::Pixel);

    CompileOptions opts;
    meshVS = CompileFromFile(L"Mesh.hlsl", "VS", ShaderType::Vertex, opts);
    meshPS = CompileFromFile(L"Mesh.hlsl", "PSForward", ShaderType::Pixel, opts);

    opts.Add("AlphaTest_", 1);
    meshAlphaTestPS = CompileFromFile(L"Mesh.hlsl", "PSForward", ShaderType::Pixel, opts);
}

// Loads resources
void MeshRenderer::Initialize(const Model* model_)
{
    model = model_;

    const uint64 numMeshes = model->Meshes().Size();
    meshBoundingBoxes.Init(numMeshes);
    frustumCulledIndices.Init(numMeshes, uint32(-1));
    meshZDepths.Init(numMeshes, FloatMax);
    for(uint64 i = 0; i < numMeshes; ++i)
    {
        const Mesh& mesh = model->Meshes()[i];
        DirectX::BoundingBox& boundingBox = meshBoundingBoxes[i];
        Float3 extents = (mesh.AABBMax() - mesh.AABBMin()) / 2.0f;
        Float3 center = mesh.AABBMin() + extents;
        boundingBox.Center = center.ToXMFLOAT3();
        boundingBox.Extents = extents.ToXMFLOAT3();
    }

    LoadShaders();

    {
        DepthBufferInit dbInit;
        dbInit.Width = SunShadowMapSize;
        dbInit.Height = SunShadowMapSize;
        dbInit.Format = DXGI_FORMAT_D32_FLOAT;
        dbInit.MSAASamples = ShadowHelper::NumMSAASamples();
        dbInit.ArraySize = NumCascades;
        dbInit.InitialState = D3D12_RESOURCE_STATE_DEPTH_READ | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        dbInit.Name = L"Sun Shadow Map";
        sunDepthMap.Initialize(dbInit);
    }

    {
        DepthBufferInit dbInit;
        dbInit.Width = SpotLightShadowMapSize;
        dbInit.Height = SpotLightShadowMapSize;
        dbInit.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        dbInit.MSAASamples = ShadowHelper::NumMSAASamples();
        dbInit.ArraySize = Max(model->SpotLights().Size(), 1ull);
        dbInit.InitialState = D3D12_RESOURCE_STATE_DEPTH_READ | D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        dbInit.Name = L"Spot Light Shadow Map";
        spotLightDepthMap.Initialize(dbInit);
    }

	{
		RenderTextureInit rtInit;
		rtInit.Width = SunShadowMapSize;
		rtInit.Height = SunShadowMapSize;
		rtInit.Format = DXGI_FORMAT_R32_FLOAT;
		rtInit.MSAASamples = ShadowHelper::NumMSAASamples();
		rtInit.ArraySize = NumCascades;
		rtInit.InitialState = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
		rtInit.Name = L"Sun Target";
		sunDepthTexture.Initialize(rtInit);
	}

    const uint64 numMaterialTextures = model->MaterialTextures().Count();

    {
        // Create a structured buffer containing texture indices per-material
        const Array<MeshMaterial>& materials = model->Materials();
        const uint64 numMaterials = materials.Size();
        Array<Material> matBufferData(numMaterials);
        for(uint64 i = 0; i < numMaterials; ++i)
        {
            Material& matIndices = matBufferData[i];
            const MeshMaterial& material = materials[i];

            matIndices.Albedo = material.Textures[uint64(MaterialTextures::Albedo)]->SRV;
            matIndices.Normal = material.Textures[uint64(MaterialTextures::Normal)]->SRV;
            matIndices.Roughness = material.Textures[uint64(MaterialTextures::Roughness)]->SRV;
            matIndices.Metallic = material.Textures[uint64(MaterialTextures::Metallic)]->SRV;
            matIndices.Emissive = material.Textures[uint64(MaterialTextures::Emissive)]->SRV;

            // Opacity is optional
            const Texture* opacity = material.Textures[uint64(MaterialTextures::Opacity)];
            matIndices.Opacity = opacity ? opacity->SRV : uint32(-1);
        }

        StructuredBufferInit sbInit;
        sbInit.Stride = sizeof(Material);
        sbInit.NumElements = numMaterials;
        sbInit.Dynamic = false;
        sbInit.InitData = matBufferData.Data();
        materialBuffer.Initialize(sbInit);
        materialBuffer.Resource()->SetName(L"Material Texture Indices");
    }

    {
        // Main pass root signature
        D3D12_ROOT_PARAMETER1 rootParameters[NumMainPassRootParams] = {};

        // "Standard"  descriptor table
        rootParameters[MainPass_StandardDescriptors].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        rootParameters[MainPass_StandardDescriptors].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_StandardDescriptors].DescriptorTable.pDescriptorRanges = DX12::StandardDescriptorRanges();
        rootParameters[MainPass_StandardDescriptors].DescriptorTable.NumDescriptorRanges = DX12::NumStandardDescriptorRanges;

        // VSCBuffer
        rootParameters[MainPass_VSCBuffer].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_VSCBuffer].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
        rootParameters[MainPass_VSCBuffer].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_VSCBuffer].Descriptor.ShaderRegister = 0;
        rootParameters[MainPass_VSCBuffer].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC;

        // PSCBuffer
        rootParameters[MainPass_PSCBuffer].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_PSCBuffer].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_PSCBuffer].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_PSCBuffer].Descriptor.ShaderRegister = 0;
        rootParameters[MainPass_PSCBuffer].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC;

        // ShadowCBuffer
        rootParameters[MainPass_ShadowCBuffer].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_ShadowCBuffer].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_ShadowCBuffer].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_ShadowCBuffer].Descriptor.ShaderRegister = 1;
        rootParameters[MainPass_ShadowCBuffer].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC;

        // MatIndexCBuffer
        rootParameters[MainPass_MatIndexCBuffer].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        rootParameters[MainPass_MatIndexCBuffer].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_MatIndexCBuffer].Constants.Num32BitValues = 1;
        rootParameters[MainPass_MatIndexCBuffer].Constants.RegisterSpace = 0;
        rootParameters[MainPass_MatIndexCBuffer].Constants.ShaderRegister = 2;

        // LightCBuffer
        rootParameters[MainPass_LightCBuffer].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_LightCBuffer].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_LightCBuffer].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_LightCBuffer].Descriptor.ShaderRegister = 3;
        rootParameters[MainPass_LightCBuffer].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC_WHILE_SET_AT_EXECUTE;

        // SRV descriptor indices
        rootParameters[MainPass_SRVIndices].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_SRVIndices].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_SRVIndices].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_SRVIndices].Descriptor.ShaderRegister = 4;
        rootParameters[MainPass_SRVIndices].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC;

        // AppSettings
        rootParameters[MainPass_AppSettings].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[MainPass_AppSettings].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParameters[MainPass_AppSettings].Descriptor.RegisterSpace = 0;
        rootParameters[MainPass_AppSettings].Descriptor.ShaderRegister = AppSettings::CBufferRegister;
        rootParameters[MainPass_AppSettings].Descriptor.Flags = D3D12_ROOT_DESCRIPTOR_FLAG_DATA_STATIC;

        D3D12_STATIC_SAMPLER_DESC staticSamplers[2] = {};
        staticSamplers[0] = DX12::GetStaticSamplerState(SamplerState::Anisotropic, 0);
        staticSamplers[1] = DX12::GetStaticSamplerState(SamplerState::ShadowMapPCF, 1);

        D3D12_ROOT_SIGNATURE_DESC1 rootSignatureDesc = {};
        rootSignatureDesc.NumParameters = ArraySize_(rootParameters);
        rootSignatureDesc.pParameters = rootParameters;
        rootSignatureDesc.NumStaticSamplers = ArraySize_(staticSamplers);
        rootSignatureDesc.pStaticSamplers = staticSamplers;
        rootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

        DX12::CreateRootSignature(&mainPassRootSignature, rootSignatureDesc);
    }

    {
        // Depth only root signature
        D3D12_ROOT_PARAMETER1 rootParameters[1] = {};

        // VSCBuffer
        rootParameters[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParameters[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_VERTEX;
        rootParameters[0].Descriptor.RegisterSpace = 0;
        rootParameters[0].Descriptor.ShaderRegister = 0;

        D3D12_ROOT_SIGNATURE_DESC1 rootSignatureDesc = {};
        rootSignatureDesc.NumParameters = ArraySize_(rootParameters);
        rootSignatureDesc.pParameters = rootParameters;
        rootSignatureDesc.NumStaticSamplers = 0;
        rootSignatureDesc.pStaticSamplers = nullptr;
        rootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

        DX12::CreateRootSignature(&depthRootSignature, rootSignatureDesc);
    }
}

void MeshRenderer::Shutdown()
{
    DestroyPSOs();
    sunDepthMap.Shutdown();
	sunDepthTexture.Shutdown();
    spotLightDepthMap.Shutdown();
    materialBuffer.Shutdown();
    DX12::Release(mainPassRootSignature);
    DX12::Release(depthRootSignature);
}

void MeshRenderer::CreatePSOs(DXGI_FORMAT mainRTFormat, DXGI_FORMAT depthFormat, uint32 numMSAASamples)
{
    if(model == nullptr)
        return;


    ID3D12Device* device = DX12::Device;

    {
        // Main pass PSO
        D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature = mainPassRootSignature;
        psoDesc.VS = meshVS.ByteCode();
        psoDesc.PS = meshPS.ByteCode();
        psoDesc.RasterizerState = DX12::GetRasterizerState(RasterizerState::BackFaceCull);
        psoDesc.BlendState = DX12::GetBlendState(BlendState::Disabled);
        psoDesc.DepthStencilState = DX12::GetDepthState(DepthState::WritesEnabled);
        psoDesc.SampleMask = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        psoDesc.NumRenderTargets = 1;
        psoDesc.RTVFormats[0] = mainRTFormat;
        psoDesc.DSVFormat = depthFormat;
        psoDesc.SampleDesc.Count = numMSAASamples;
        psoDesc.SampleDesc.Quality = numMSAASamples > 1 ? DX12::StandardMSAAPattern : 0;
        psoDesc.InputLayout.NumElements = uint32(Model::NumInputElements());
        psoDesc.InputLayout.pInputElementDescs = Model::InputElements();
        DXCall(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&mainPassPSO)));

        psoDesc.PS = meshAlphaTestPS.ByteCode();
        DXCall(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&mainPassAlphaTestPSO)));
    }

    {
        // Depth-only PSO
        D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature = depthRootSignature;
        psoDesc.VS = meshDepthVS.ByteCode();
		psoDesc.PS = meshDepthPS.ByteCode();
        psoDesc.RasterizerState = DX12::GetRasterizerState(RasterizerState::BackFaceCull);
        psoDesc.BlendState = DX12::GetBlendState(BlendState::DisabledRed);
        psoDesc.DepthStencilState = DX12::GetDepthState(DepthState::WritesEnabled);
        psoDesc.SampleMask = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        psoDesc.NumRenderTargets = 1;
        psoDesc.DSVFormat = depthFormat;
		psoDesc.RTVFormats[0] = DXGI_FORMAT_R32_FLOAT;
        psoDesc.SampleDesc.Count = numMSAASamples;
        psoDesc.SampleDesc.Quality = numMSAASamples > 1 ? DX12::StandardMSAAPattern : 0;
        psoDesc.InputLayout.NumElements = uint32(Model::NumInputElements());
        psoDesc.InputLayout.pInputElementDescs = Model::InputElements();
        DXCall(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&depthPSO)));

        // Spotlight shadow depth PSO
        psoDesc.DSVFormat = spotLightDepthMap.DSVFormat;
        psoDesc.SampleDesc.Count = spotLightDepthMap.MSAASamples;
        psoDesc.SampleDesc.Quality = spotLightDepthMap.MSAASamples > 1 ? DX12::StandardMSAAPattern : 0;
        psoDesc.RasterizerState = DX12::GetRasterizerState(RasterizerState::BackFaceCull);
        DXCall(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&spotLightShadowPSO)));

        // Sun shadow depth PSO
        psoDesc.DSVFormat = sunDepthMap.DSVFormat;
        psoDesc.SampleDesc.Count = sunDepthMap.MSAASamples;
        psoDesc.SampleDesc.Quality = sunDepthMap.MSAASamples > 1 ? DX12::StandardMSAAPattern : 0;
        psoDesc.RasterizerState = DX12::GetRasterizerState(RasterizerState::BackFaceCullNoZClip);
        DXCall(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&sunShadowPSO)));
    }
}

void MeshRenderer::DestroyPSOs()
{
    DX12::DeferredRelease(mainPassPSO);
    DX12::DeferredRelease(mainPassAlphaTestPSO);
    DX12::DeferredRelease(depthPSO);
    DX12::DeferredRelease(spotLightShadowPSO);
    DX12::DeferredRelease(sunShadowPSO);
}

// Renders all meshes in the model, with shadows
void MeshRenderer::RenderMainPass(ID3D12GraphicsCommandList* cmdList, const Camera& camera, const MainPassData& mainPassData)
{
    PIXMarker marker(cmdList, "Mesh Rendering");

    const uint64 numVisible = CullMeshes(camera, meshBoundingBoxes, frustumCulledIndices);
    const uint32* meshDrawIndices = frustumCulledIndices.Data();

    cmdList->SetGraphicsRootSignature(mainPassRootSignature);
    cmdList->SetPipelineState(mainPassPSO);
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    ID3D12PipelineState* currPSO = mainPassPSO;

    DX12::BindStandardDescriptorTable(cmdList, MainPass_StandardDescriptors, CmdListMode::Graphics);

    Float4x4 world;

    // Set constant buffers
    MeshVSConstants vsConstants;
    vsConstants.World = world;
    vsConstants.View = camera.ViewMatrix();
    vsConstants.WorldViewProjection = world * camera.ViewProjectionMatrix();
    DX12::BindTempConstantBuffer(cmdList, vsConstants, MainPass_VSCBuffer, CmdListMode::Graphics);

    ShadingConstants psConstants;
    psConstants.SunDirectionWS = AppSettings::SunDirection;
    psConstants.SunIrradiance = mainPassData.SkyCache->SunIrradiance;
    psConstants.CosSunAngularRadius = std::cos(DegToRad(AppSettings::SunSize));
    psConstants.SinSunAngularRadius = std::sin(DegToRad(AppSettings::SunSize));
    psConstants.CameraPosWS = camera.Position();

    psConstants.NumXTiles = uint32(AppSettings::NumXTiles);
    psConstants.NumXYTiles = uint32(AppSettings::NumXTiles * AppSettings::NumYTiles);
    psConstants.NearClip = camera.NearClip();
    psConstants.FarClip = camera.FarClip();

    psConstants.SkySH = mainPassData.SkyCache->SH;
    DX12::BindTempConstantBuffer(cmdList, psConstants, MainPass_PSCBuffer, CmdListMode::Graphics);

    DX12::BindTempConstantBuffer(cmdList, sunShadowConstants, MainPass_ShadowCBuffer, CmdListMode::Graphics);

    mainPassData.SpotLightBuffer->SetAsGfxRootParameter(cmdList, MainPass_LightCBuffer);

    AppSettings::BindCBufferGfx(cmdList, MainPass_AppSettings);

    uint32 psSRVs[] =
    {
		sunDepthTexture.SRV(),
        spotLightDepthMap.SRV(),
        materialBuffer.SRV,
        mainPassData.SpotLightClusterBuffer->SRV,
    };

    DX12::BindTempConstantBuffer(cmdList, psSRVs, MainPass_SRVIndices, CmdListMode::Graphics);

    // Bind vertices and indices
    D3D12_VERTEX_BUFFER_VIEW vbView = model->VertexBuffer().VBView();
    D3D12_INDEX_BUFFER_VIEW ibView = model->IndexBuffer().IBView();
    cmdList->IASetVertexBuffers(0, 1, &vbView);
    cmdList->IASetIndexBuffer(&ibView);

    // Draw all visible meshes
    uint32 currMaterial = uint32(-1);
    for(uint64 i = 0; i < numVisible; ++i)
    {
        uint64 meshIdx = meshDrawIndices[i];
        const Mesh& mesh = model->Meshes()[meshIdx];

        // Draw all parts
        for(uint64 partIdx = 0; partIdx < mesh.NumMeshParts(); ++partIdx)
        {
            const MeshPart& part = mesh.MeshParts()[partIdx];
            if(part.MaterialIdx != currMaterial)
            {
                cmdList->SetGraphicsRoot32BitConstant(MainPass_MatIndexCBuffer, part.MaterialIdx, 0);
                currMaterial = part.MaterialIdx;
            }

            ID3D12PipelineState* newPSO = mainPassPSO;
            const MeshMaterial& material = model->Materials()[part.MaterialIdx];
            if(material.Textures[uint64(MaterialTextures::Opacity)] != nullptr)
                newPSO = mainPassAlphaTestPSO;

            if(currPSO != newPSO)
            {
                cmdList->SetPipelineState(newPSO);
                currPSO = newPSO;
            }

            cmdList->DrawIndexedInstanced(part.IndexCount, 1, mesh.IndexOffset() + part.IndexStart, mesh.VertexOffset(), 0);
        }
    }
}

// Renders all meshes using depth-only rendering
void MeshRenderer::RenderDepth(ID3D12GraphicsCommandList* cmdList, const Camera& camera, ID3D12PipelineState* pso, uint64 numVisible, const uint32* meshDrawIndices)
{
    cmdList->SetGraphicsRootSignature(depthRootSignature);
    cmdList->SetPipelineState(pso);
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    Float4x4 world;

    // Set constant buffers
    MeshVSConstants vsConstants;
    vsConstants.World = world;
    vsConstants.View = camera.ViewMatrix();
    vsConstants.WorldViewProjection = world * camera.ViewProjectionMatrix();
    DX12::BindTempConstantBuffer(cmdList, vsConstants, 0, CmdListMode::Graphics);

    // Bind vertices and indices
    D3D12_VERTEX_BUFFER_VIEW vbView = model->VertexBuffer().VBView();
    D3D12_INDEX_BUFFER_VIEW ibView = model->IndexBuffer().IBView();
    cmdList->IASetVertexBuffers(0, 1, &vbView);
    cmdList->IASetIndexBuffer(&ibView);

    // Draw all meshes
    for(uint64 i = 0; i < numVisible; ++i)
    {
        uint64 meshIdx = meshDrawIndices[i];
        const Mesh& mesh = model->Meshes()[meshIdx];

        // Draw the whole mesh
        cmdList->DrawIndexedInstanced(mesh.NumIndices(), 1, mesh.IndexOffset(), mesh.VertexOffset(), 0);
    }
}

// Renders all meshes using depth-only rendering for a sun shadow map
void MeshRenderer::RenderSunShadowDepth(ID3D12GraphicsCommandList* cmdList, const OrthographicCamera& camera)
{
	uint64 timer = Profiler::GlobalProfiler.StartCPUProfile("Sun Shadow Culling");
    const uint64 numVisible = CullMeshesOrthographic(camera, true, meshBoundingBoxes, frustumCulledIndices);
	Profiler::GlobalProfiler.EndCPUProfile(timer);

	CPUProfileBlock cpuProfileBlock("Sun Shadow Depth");
    RenderDepth(cmdList, camera, sunShadowPSO, numVisible, frustumCulledIndices.Data());
}

void MeshRenderer::RenderSpotLightShadowDepth(ID3D12GraphicsCommandList* cmdList, const Camera& camera)
{
    const uint64 numVisible = CullMeshes(camera, meshBoundingBoxes, frustumCulledIndices);
    RenderDepth(cmdList, camera, spotLightShadowPSO, numVisible, frustumCulledIndices.Data());
}

// Renders meshes using cascaded shadow mapping
void MeshRenderer::RenderSunShadowMap(ID3D12GraphicsCommandList* cmdList, const Camera& camera)
{
    PIXMarker marker(cmdList, L"Sun Shadow Map Rendering");
    CPUProfileBlock cpuProfileBlock("Sun Shadow Map Rendering");
    ProfileBlock profileBlock(cmdList, "Sun Shadow Map Rendering");

    OrthographicCamera cascadeCameras[NumCascades];
    ShadowHelper::PrepareCascades(AppSettings::SunDirection, SunShadowMapSize, true, camera, sunShadowConstants.Base, cascadeCameras);

    // Transition all of the cascade array slices to a writable state
    sunDepthMap.MakeWritable(cmdList);
	sunDepthTexture.MakeWritable(cmdList);

	ID3D12GraphicsCommandList5* vrsCmdList = static_cast<ID3D12GraphicsCommandList5*>(cmdList);
	int32 quality = AppSettings::ShadowQuality;
	switch (quality)
	{
		case 0:
			vrsCmdList->RSSetShadingRate(D3D12_SHADING_RATE_1X1, nullptr);
			break;
		case 1:
			vrsCmdList->RSSetShadingRate(D3D12_SHADING_RATE_2X2, nullptr);
			break;
		case 2:
			vrsCmdList->RSSetShadingRate(D3D12_SHADING_RATE_4X4, nullptr);
			break;
		default:
			vrsCmdList->RSSetShadingRate(D3D12_SHADING_RATE_1X1, nullptr);
			break;
	}

    // Render the meshes to each cascade
    for(uint64 cascadeIdx = 0; cascadeIdx < NumCascades; ++cascadeIdx)
    {
        PIXMarker cascadeMarker(cmdList, MakeString(L"Rendering Shadow Map Cascade %u", cascadeIdx).c_str());

        // Set the viewport
        DX12::SetViewport(cmdList, SunShadowMapSize, SunShadowMapSize);

		float color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

        // Set the shadow map as the depth target
        D3D12_CPU_DESCRIPTOR_HANDLE dsv = sunDepthMap.ArrayDSVs[cascadeIdx];
		D3D12_CPU_DESCRIPTOR_HANDLE rtv = sunDepthTexture.ArrayRTVs[cascadeIdx];
        cmdList->OMSetRenderTargets(1, &rtv, false, &dsv);
		cmdList->ClearRenderTargetView(rtv, color, 0, 0);
        cmdList->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

        // Draw the mesh with depth only, using the new shadow camera
        OrthographicCamera& cascadeCam = cascadeCameras[cascadeIdx];
        RenderSunShadowDepth(cmdList, cascadeCam);
    }

	vrsCmdList->RSSetShadingRate(D3D12_SHADING_RATE_1X1, nullptr);

    sunDepthMap.MakeReadable(cmdList);
	sunDepthTexture.MakeReadable(cmdList);
}

// Render shadows for all spot lights
void MeshRenderer::RenderSpotLightShadowMap(ID3D12GraphicsCommandList* cmdList, const Camera& camera)
{
    const Array<ModelSpotLight>& spotLights = model->SpotLights();
    const uint64 numSpotLights = Min<uint64>(spotLights.Size(), AppSettings::MaxLightClamp);
    if(numSpotLights == 0)
        return;

    PIXMarker marker(cmdList, L"Spot Light Shadow Map Rendering");
    CPUProfileBlock cpuProfileBlock("Spot Light Shadow Map Rendering");
    ProfileBlock profileBlock(cmdList, "Spot Light Shadow Map Rendering");

    // Transition all of the shadow array slices to a writable state
    spotLightDepthMap.MakeWritable(cmdList);

    for(uint64 i = 0; i < numSpotLights; ++i)
    {
        PIXMarker lightMarker(cmdList, MakeString(L"Rendering Spot Light Shadow %u", i).c_str());

        // Set the viewport
        DX12::SetViewport(cmdList, SpotLightShadowMapSize, SpotLightShadowMapSize);

        // Set the shadow map as the depth target
        D3D12_CPU_DESCRIPTOR_HANDLE dsv = numSpotLights > 1 ? spotLightDepthMap.ArrayDSVs[i] : spotLightDepthMap.DSV;
        cmdList->OMSetRenderTargets(0, nullptr, false, &dsv);
        cmdList->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

        const ModelSpotLight& light = spotLights[i];

        // Draw the mesh with depth only, using the new shadow camera
        PerspectiveCamera shadowCamera;
        shadowCamera.Initialize(1.0f, light.AngularAttenuation.y, AppSettings::SpotShadowNearClip, AppSettings::SpotLightRange);
        shadowCamera.SetPosition(light.Position);
        shadowCamera.SetOrientation(light.Orientation);
        RenderSpotLightShadowDepth(cmdList, shadowCamera);

        Float4x4 shadowMatrix = shadowCamera.ViewProjectionMatrix() * ShadowHelper::ShadowScaleOffsetMatrix;
        spotLightShadowMatrices[i] = Float4x4::Transpose(shadowMatrix);
    }

    spotLightDepthMap.MakeReadable(cmdList);
}