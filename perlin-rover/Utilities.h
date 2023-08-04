#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyActivationListener.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h"
#include "Jolt/Physics/Collision/ContactListener.h"

namespace JPHUtil
{
	using namespace JPH;

	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
	static constexpr JPH::ObjectLayer NUM_LAYERS = 2;

	static constexpr BroadPhaseLayer BP_NON_MOVING(0);
	static constexpr BroadPhaseLayer BP_MOVING(1);
	static constexpr uint BP_NUM_LAYERS(2);

	class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
	{
	public:
		BPLayerInterfaceImpl();
		virtual uint GetNumBroadPhaseLayers() const override;
		virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override;
		virtual const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override;
	private:
		BroadPhaseLayer mObjectToBroadPhase[NUM_LAYERS];
	};

	class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
	{
	public:
		virtual bool ShouldCollide(ObjectLayer inObjectLayer, BroadPhaseLayer inBroadPhaseLayer) const override;
	};

	class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
	{
	public:
		virtual bool ShouldCollide(ObjectLayer inObjectLayer1, ObjectLayer inObjectLayer2) const override;
	};

	class VehicleBodyActivationListener : public BodyActivationListener
	{
	public:
		virtual void OnBodyActivated(const BodyID& inBody, uint64 inBodyUserData) override;
		virtual void OnBodyDeactivated(const BodyID& inBody, uint64 inBodyUserData) override;
	};

	class VehicleContactListener : public ContactListener
	{
		virtual ValidateResult OnContactValidate(const Body& inBody1, const Body& inBody2, RVec3Arg inBaseOffset, const CollideShapeResult& inCollisionResult) override;
		virtual void OnContactAdded(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings) override;
		virtual void OnContactPersisted(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings) override;
		virtual void OnContactRemoved(const SubShapeIDPair& inSubShapePair) override;
	};
}

namespace OBJUtil
{
	struct Vertex
	{
		float x, y, z;
	};

	struct UV
	{
		float u, v;
	};

	struct Normal
	{
		float nx, ny, nz;
	};

	struct Triangle
	{
		int v1, v2, v3;
		int uv1, uv2, uv3;
		int n1, n2, n3;
	};

	class OBJExporter
	{
	public:
		static void SaveToObj(const std::vector<Vertex>& vertices, 
			const std::vector<UV>& uvs, const std::vector<Normal>& normals,
			const std::vector<Triangle>& triangles, const std::string& filename);
	};
}