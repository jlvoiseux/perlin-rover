#include "VehicleHandler.h"
#include <iostream>

VehicleHandler::VehicleHandler(JPH::PhysicsSystem& ph)
{
    ph.SetBodyActivationListener(&mVehicleBodyActivationListener);
    ph.SetContactListener(&mVehicleContactListener);


    JPH::Vec3 wheelPosition[] =
    {
        JPH::Vec3(-1.75 * mHalfVehicleWidth, -mHalfVehicleHeight, mHalfVehicleLength - 2.0f * mHalfWheelHeight),
        JPH::Vec3(1.75 * mHalfVehicleWidth, -mHalfVehicleHeight, mHalfVehicleLength - 2.0f * mHalfWheelHeight),
        JPH::Vec3(-1.75 * mHalfVehicleWidth, -mHalfVehicleHeight, -mHalfVehicleLength - 1.0f * mHalfWheelHeight),
        JPH::Vec3(1.75 * mHalfVehicleWidth, -mHalfVehicleHeight, -mHalfVehicleLength - 1.0f * mHalfWheelHeight),
    };

    JPH::RVec3 position(START_X, START_Y, START_Z);
    JPH::Ref<JPH::BoxShape> bodyShape = new JPH::BoxShape(JPH::Vec3(mHalfVehicleWidth, mHalfVehicleHeight, mHalfVehicleLength));
    JPH::Ref<JPH::CylinderShape> wheelShape = new JPH::CylinderShape(mHalfWheelWidth, mHalfWheelHeight);
    wheelShape->SetDensity(1.0e4f);

    JPH::Ref<JPH::GroupFilterTable> groupFilter = new JPH::GroupFilterTable;
    mChassis = ph.GetBodyInterface().CreateBody(JPH::BodyCreationSettings(bodyShape, position, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, JPHUtil::MOVING));
    mChassis->SetCollisionGroup(JPH::CollisionGroup(groupFilter, 0, 0));
    ph.GetBodyInterface().AddBody(mChassis->GetID(), JPH::EActivation::Activate);

    for (int i = 0; i < (int)EWheel::Num; ++i)
    {
        bool isFront = sIsFrontWheel((EWheel)i);
        bool isLeft = sIsLeftWheel((EWheel)i);

        JPH::RVec3 wheelPos0 = position + wheelPosition[i];
        JPH::RVec3 wheelPos1 = wheelPos0 - JPH::Vec3(0, mHalfWidthTravel, 0);

        // Create body
        JPH::Body& wheel = *ph.GetBodyInterface().CreateBody(JPH::BodyCreationSettings(wheelShape, wheelPos1, JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f * JPH::JPH_PI), JPH::EMotionType::Dynamic, JPHUtil::MOVING));
        wheel.SetFriction(1.0f);
        wheel.SetCollisionGroup(JPH::CollisionGroup(groupFilter, 0, 0));
        ph.GetBodyInterface().AddBody(wheel.GetID(), JPH::EActivation::Activate);

        // Create constraint
        JPH::SixDOFConstraintSettings settings;
        settings.mPosition1 = wheelPos0;
        settings.mPosition2 = wheelPos1;
        settings.mAxisX1 = settings.mAxisX2 = isLeft ? -JPH::Vec3::sAxisX() : JPH::Vec3::sAxisX();
        settings.mAxisY1 = settings.mAxisY2 = JPH::Vec3::sAxisY();

        // The suspension works in the Y translation axis only
        settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationX);
        settings.SetLimitedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationY, -mHalfWidthTravel, mHalfWidthTravel);
        settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::TranslationZ);
        settings.mMotorSettings[JPH::SixDOFConstraintSettings::EAxis::TranslationY] = JPH::MotorSettings(2.0f, 1.0f, 1.0e5f, 0.0f);

        // Front wheel can rotate around the Y axis
        if (isFront)
            settings.SetLimitedAxis(JPH::SixDOFConstraintSettings::EAxis::RotationY, -cMaxSteeringAngle, cMaxSteeringAngle);
        else
            settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::RotationY);

        // The Z axis is static
        settings.MakeFixedAxis(JPH::SixDOFConstraintSettings::EAxis::RotationZ);

        // The main engine drives the X axis
        settings.MakeFreeAxis(JPH::SixDOFConstraintSettings::EAxis::RotationX);
        settings.mMotorSettings[JPH::SixDOFConstraintSettings::EAxis::RotationX] = JPH::MotorSettings(2.0f, 1.0f, 0.0f, 0.5e4f);

        // The front wheel needs to be able to steer around the Y axis
        // However the motors work in the constraint space of the wheel, and since this rotates around the 
        // X axis we need to drive both the Y and Z to steer
        if (isFront)
            settings.mMotorSettings[JPH::SixDOFConstraintSettings::EAxis::RotationY] = settings.mMotorSettings[JPH::SixDOFConstraintSettings::EAxis::RotationZ] = JPH::MotorSettings(10.0f, 1.0f, 0.0f, 1.0e6f);

        JPH::SixDOFConstraint* wheelConstraint = static_cast<JPH::SixDOFConstraint*>(settings.Create(*mChassis, wheel));
        ph.AddConstraint(wheelConstraint);
        mWheels[i] = wheelConstraint;

        // Drive the suspension
        wheelConstraint->SetTargetPositionCS(JPH::Vec3(0, -mHalfWidthTravel, 0));
        wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::TranslationY, JPH::EMotorState::Position);

        // The front wheels steer around the Y axis, but in constraint space of the wheel this means we need to drive
        // both Y and Z (see comment above)
        if (isFront)
        {
            wheelConstraint->SetTargetOrientationCS(JPH::Quat::sIdentity());
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationY, JPH::EMotorState::Position);
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::RotationZ, JPH::EMotorState::Position);
        }
    }

    mWheelFwdLeftModel = LoadModelFromMesh(GenMeshCylinder(mHalfWheelHeight, 2 * mHalfWheelWidth, 32));
	mWheelFwdRightModel = LoadModelFromMesh(GenMeshCylinder(mHalfWheelHeight, 2 * mHalfWheelWidth, 32));
	mWheelRearLeftModel = LoadModelFromMesh(GenMeshCylinder(mHalfWheelHeight, 2 * mHalfWheelWidth, 32));
	mWheelRearRightModel = LoadModelFromMesh(GenMeshCylinder(mHalfWheelHeight, 2 * mHalfWheelWidth, 32));
	mChassisModel = LoadModelFromMesh(GenMeshCube(mHalfVehicleWidth * 2, mHalfVehicleHeight * 2, mHalfVehicleLength * 2.0f));
}


void VehicleHandler::Update(JPH::PhysicsSystem& ph, bool fwdFlag, bool bwdFlag, bool lftFlag, bool rhtFlag)
{
    float steeringAngle = 0.0f, speed = 0.0f;
    if (lftFlag)		steeringAngle = cMaxSteeringAngle;
    if (rhtFlag)	    steeringAngle = -cMaxSteeringAngle;
    if (fwdFlag)		speed = cMaxRotationSpeed;
    if (bwdFlag)		speed = -cMaxRotationSpeed;

    // On user input, assure that the car is active
    if (steeringAngle != 0.0f || speed != 0.0f)
        ph.GetBodyInterface().ActivateBody(mChassis->GetID());

    // Brake if current velocity is in the opposite direction of the desired velocity
    float carSpeed = mChassis->GetLinearVelocity().Dot(mChassis->GetRotation().RotateAxisZ());
    bool brake = speed != 0.0f && carSpeed != 0.0f && JPH::Sign(speed) != JPH::Sign(carSpeed);

    // Front wheels
    const EWheel frontWheels[] = { EWheel::FrontLeft, EWheel::FrontRight };
    for (EWheel w : frontWheels)
    {
        JPH::SixDOFConstraint* wheelConstraint = mWheels[(int)w];
        if (wheelConstraint == nullptr)
            continue;

        // Steer front wheels
        JPH::Quat steeringRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), steeringAngle);
        wheelConstraint->SetTargetOrientationCS(steeringRotation);

        if (brake)
        {
            // Brake on front wheels
            wheelConstraint->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::EMotorState::Velocity);
        }
        else if (speed != 0.0f)
        {
            // Front wheel drive, since the motors are applied in the constraint space of the wheel
            // it is always applied on the X axis
            wheelConstraint->SetTargetAngularVelocityCS(JPH::Vec3(sIsLeftWheel(w) ? -speed : speed, 0, 0));
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::EMotorState::Velocity);
        }
        else
        {
            // Free spin
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::EMotorState::Off);
        }
    }

    // Rear wheels
    const EWheel rearWheels[] = { EWheel::RearLeft, EWheel::RearRight };
    for (EWheel w : rearWheels)
    {
        JPH::SixDOFConstraint* wheelConstraint = mWheels[(int)w];
        if (wheelConstraint == nullptr)
            continue;

        if (brake)
        {
            // Brake on rear wheels
            wheelConstraint->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::EMotorState::Velocity);
        }
        else
        {
            // Free spin
            wheelConstraint->SetMotorState(JPH::SixDOFConstraintSettings::EAxis::RotationX, JPH::EMotorState::Off);
        }
    }
}


void VehicleHandler::DrawVehicle(
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
)
{
    Vector3 outAxis = Vector3One();
    float outAngle = 0.0f;
    
    Vector3 wheelFwdLeftPos = Vector3{ wheelFwdLeftPosArg.GetX(), wheelFwdLeftPosArg.GetY() , wheelFwdLeftPosArg.GetZ() };
	Quaternion wheelFwdLeftQuat = Quaternion{ wheelFwdLeftRotArg.GetW(), wheelFwdLeftRotArg.GetX(), wheelFwdLeftRotArg.GetY(), wheelFwdLeftRotArg.GetZ() };
    QuaternionToAxisAngle(wheelFwdLeftQuat, &outAxis, &outAngle);
    mWheelFwdLeftModel.transform = QuaternionToMatrix(QuaternionFromAxisAngle(Vector3{ outAxis.z, outAxis.y, outAxis.x }, outAngle));
    DrawModelWires(mWheelFwdLeftModel, wheelFwdLeftPos, 1.0f, SKYBLUE);

    Vector3 wheelFwdRightPos = Vector3{ wheelFwdRightPosArg.GetX(), wheelFwdRightPosArg.GetY() , wheelFwdRightPosArg.GetZ() };
	Quaternion wheelFwdRightQuat = Quaternion{ wheelFwdRightRotArg.GetW(), wheelFwdRightRotArg.GetX(), wheelFwdRightRotArg.GetY(), wheelFwdRightRotArg.GetZ() };
	QuaternionToAxisAngle(wheelFwdRightQuat, &outAxis, &outAngle);
	mWheelFwdRightModel.transform = QuaternionToMatrix(QuaternionFromAxisAngle(Vector3{ outAxis.z, outAxis.y, outAxis.x }, outAngle));
	DrawModelWires(mWheelFwdRightModel, wheelFwdRightPos, 1.0f, SKYBLUE);
    
    Vector3 wheelRearLeftPos = Vector3{ wheelRearLeftPosArg.GetX(), wheelRearLeftPosArg.GetY() , wheelRearLeftPosArg.GetZ() };
	Quaternion wheelRearLeftQuat = Quaternion{ wheelRearLeftRotArg.GetW(), wheelRearLeftRotArg.GetX(), wheelRearLeftRotArg.GetY(), wheelRearLeftRotArg.GetZ() };
	QuaternionToAxisAngle(wheelRearLeftQuat, &outAxis, &outAngle);
	mWheelRearLeftModel.transform = QuaternionToMatrix(QuaternionFromAxisAngle(Vector3{ outAxis.z, outAxis.y, outAxis.x }, outAngle));
	DrawModelWires(mWheelRearLeftModel, wheelRearLeftPos, 1.0f, SKYBLUE);
    
    Vector3 wheelRearRightPos = Vector3{ wheelRearRightPosArg.GetX(), wheelRearRightPosArg.GetY() , wheelRearRightPosArg.GetZ() };
	Quaternion wheelRearRightQuat = Quaternion{ wheelRearRightRotArg.GetW(), wheelRearRightRotArg.GetX(), wheelRearRightRotArg.GetY(), wheelRearRightRotArg.GetZ() };
	QuaternionToAxisAngle(wheelRearRightQuat, &outAxis, &outAngle);
	mWheelRearRightModel.transform = QuaternionToMatrix(QuaternionFromAxisAngle(Vector3{ outAxis.z, outAxis.y, outAxis.x }, outAngle));
	DrawModelWires(mWheelRearRightModel, wheelRearRightPos, 1.0f, SKYBLUE);
    
    Vector3 chassisPos = Vector3{ chassisPosArg.GetX(), chassisPosArg.GetY() , chassisPosArg.GetZ() };
    Quaternion chassisQuat = Quaternion{ chassisRotArg.GetW(), chassisRotArg.GetX(), chassisRotArg.GetY(), chassisRotArg.GetZ() };
    QuaternionToAxisAngle(chassisQuat, &outAxis, &outAngle);
	mChassisModel.transform = QuaternionToMatrix(QuaternionFromAxisAngle(Vector3{ -outAxis.z, outAxis.y, -outAxis.x }, outAngle));
	DrawModelWires(mChassisModel, chassisPos, 1.0f, RED);

    cameraPositionAnchorPoint = Vector3Add(Vector3Scale(Vector3Add(wheelRearLeftPos, wheelRearRightPos), 0.5f), Vector3{ 0, 3, -5 });
    cameraTargetAnchorPoint = Vector3Scale(Vector3Add(wheelFwdLeftPos, wheelFwdRightPos), 0.5f);
  }