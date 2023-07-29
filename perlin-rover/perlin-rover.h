#pragma once
#include "raylib.h"

#include "Jolt/Jolt.h"
#include "Jolt/RegisterTypes.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/TempAllocator.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/Physics/EActivation.h"
#include "Jolt/Physics/PhysicsSettings.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyActivationListener.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h"
#include "Jolt/Physics/Collision/GroupFilterTable.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Constraints/SixDOFConstraint.h"


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


class KeyHandler
{
public:
	void update();

	bool Fwd = false;
	bool Bwd = false;
	bool Lft = false;
	bool Rht = false;

private:
	void KeyPressed();
	void KeyReleased();
	float mSensitivity = 0.1f;
	float mSpeed = 1.0f;
};

struct Terrain
{
	JPH::Body* physicsTerrain;
	Mesh* meshTerrain;
};

class TerrainHandler
{
public:
	TerrainHandler(JPH::PhysicsSystem& ph);
	~TerrainHandler();
	void DrawTerrain();
private:
	Terrain* GenerateTerrain(JPH::PhysicsSystem& ph);
	void CreateTerrainTriangle(Mesh &mesh, JPH::Float3 v0, JPH::Float3 v1, JPH::Float3 v2, int vertexIndex, int texCoordIndex);
	
	Terrain* mTerrain;
	Model mTerrainModel;
};


class VehicleHandler
{
public:
	enum class EWheel : int { FrontLeft, FrontRight, RearLeft, RearRight, Num };

	VehicleHandler(JPH::PhysicsSystem &ph);
	void Update(JPH::PhysicsSystem &ph, bool fwdFlag, bool bwdFlag, bool lftFlag, bool rhtFlag);
	JPH::Body* GetChassis() { return mChassis; }
	JPH::SixDOFConstraint* GetWheel(EWheel wheelType) { return mWheels[int(wheelType)]; }
	
	void DrawVehicle(
		JPH::RVec3 wheelFwdLeftPos,
		JPH::Quat wheelFwdLeftRot,
		JPH::RVec3 wheelFwdRightPos,
		JPH::Quat wheelFwdRightRot,
		JPH::RVec3 wheelRearLeftPos,
		JPH::Quat wheelRearLeftRot,
		JPH::RVec3 wheelRearRightPos,
		JPH::Quat wheelRearRightRot,
		JPH::RVec3 chassisPos,
		JPH::Quat chassisRot
	);

private:
	static inline bool		sIsFrontWheel(EWheel inWheel) { return inWheel == EWheel::FrontLeft || inWheel == EWheel::FrontRight; }
	static inline bool		sIsLeftWheel(EWheel inWheel) { return inWheel == EWheel::FrontLeft || inWheel == EWheel::RearLeft; }

	static constexpr float cMaxSteeringAngle = JPH::DegreesToRadians(30);
	static constexpr float cMaxRotationSpeed = 100.0f * JPH::JPH_PI;
	JPH::Body* mChassis;

	JPHUtil::VehicleBodyActivationListener mVehicleBodyActivationListener;
	JPHUtil::VehicleContactListener mVehicleContactListener;

	JPH::Ref<JPH::SixDOFConstraint> mWheels[int(EWheel::Num)];

	const float halfVehicleLength = 2.0f;
	const float halfVehicleWidth = 0.9f;
	const float halfVehicleHeight = 0.2f;

	const float halfWheelHeight = 0.3f;
	const float halfWheelWidth = 0.05f;
	const float halfWidthTravel = 0.5f;
};
