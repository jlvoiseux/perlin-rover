#pragma once

#include "Falcor.h"
#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/GroupFilterTable.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Constraints/SixDOFConstraint.h"
#include "Jolt/Physics/PhysicsSystem.h"

#include "Utilities.h"

#define START_X 25
#define START_Y 250
#define START_Z 25

class VehicleHandler
{
public:
	enum class EWheel : int { FrontLeft, FrontRight, RearLeft, RearRight, Num };

	VehicleHandler(JPH::PhysicsSystem& ph);
	void Update(JPH::PhysicsSystem& ph, bool fwdFlag, bool bwdFlag, bool lftFlag, bool rhtFlag);
	JPH::Body* GetChassis() { return mChassis; }
	JPH::SixDOFConstraint* GetWheel(EWheel wheelType) { return mWheels[int(wheelType)]; }

	void UpdateVehicle(
		JPH::RVec3 wheelFwdLeftPosArg,
		JPH::Quat wheelFwdLeftRotArg,
		JPH::RVec3 wheelFwdRightPosArg,
		JPH::Quat wheelFwdRightRotArg,
		JPH::RVec3 wheelRearLeftPosArg,
		JPH::Quat wheelRearLeftRotArg,
		JPH::RVec3 wheelRearRightPosArg,
		JPH::Quat wheelRearRightRotArg,
		JPH::RVec3 chassisPosArg,
		JPH::Quat chassisRotArg
	);

	Falcor::Transform* GetPartTransform(EWheel wheelType)
	{
		switch (wheelType)
		{
		case EWheel::FrontLeft:
			return &mWheelFwdLeftTransform;
		case EWheel::FrontRight:
			return &mWheelFwdRightTransform;
		case EWheel::RearLeft:
			return &mWheelRearLeftTransform;
		case EWheel::RearRight:
			return &mWheelRearRightTransform;
		default:
			return &mChassisTransform;
		}
	}

	Falcor::float3 cameraAnchor;
	Falcor::float3 cameraTargetAnchor;

private:
	static inline bool		sIsFrontWheel(EWheel inWheel) { return inWheel == EWheel::FrontLeft || inWheel == EWheel::FrontRight; }
	static inline bool		sIsLeftWheel(EWheel inWheel) { return inWheel == EWheel::FrontLeft || inWheel == EWheel::RearLeft; }

	static constexpr float cMaxSteeringAngle = JPH::DegreesToRadians(30);
	static constexpr float cMaxRotationSpeed = 100.0f * JPH::JPH_PI;
	JPH::Body* mChassis;

	JPHUtil::VehicleBodyActivationListener mVehicleBodyActivationListener;
	JPHUtil::VehicleContactListener mVehicleContactListener;

	JPH::Ref<JPH::SixDOFConstraint> mWheels[int(EWheel::Num)];

	const float mHalfVehicleLength = 2.5f;
	const float mHalfVehicleWidth = 0.8f;
	const float mHalfVehicleHeight = 0.6f;

	const float mHalfWheelHeight = 0.5f;
	const float mHalfWheelWidth = 0.15f;
	const float mHalfWidthTravel = 0.2f;

	Falcor::Transform mWheelFwdLeftTransform;
	Falcor::Transform mWheelFwdRightTransform;
	Falcor::Transform mWheelRearLeftTransform;
	Falcor::Transform mWheelRearRightTransform;
	Falcor::Transform mChassisTransform;	
};