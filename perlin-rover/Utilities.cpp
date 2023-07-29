#include "Utilities.h"

#include <iostream>

JPHUtil::BPLayerInterfaceImpl::BPLayerInterfaceImpl()
{
    mObjectToBroadPhase[NON_MOVING] = BP_NON_MOVING;
    mObjectToBroadPhase[MOVING] = BP_MOVING;
}


JPH::uint JPHUtil::BPLayerInterfaceImpl::GetNumBroadPhaseLayers() const
{
    return BP_NUM_LAYERS;
}


JPH::BroadPhaseLayer JPHUtil::BPLayerInterfaceImpl::GetBroadPhaseLayer(ObjectLayer inLayer) const
{
    return mObjectToBroadPhase[inLayer];
}


const char* JPHUtil::BPLayerInterfaceImpl::GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const
{
    switch ((BroadPhaseLayer::Type)inLayer)
    {
    case (BroadPhaseLayer::Type)JPHUtil::NON_MOVING:	return "NON_MOVING";
    case (BroadPhaseLayer::Type)JPHUtil::MOVING:	return "MOVING";
    default: return "INVALID";
    }
}


bool JPHUtil::ObjectVsBroadPhaseLayerFilterImpl::ShouldCollide(ObjectLayer inObjectLayer, BroadPhaseLayer inBroadPhaseLayer) const
{
    switch (inObjectLayer)
    {
    case NON_MOVING:
        return inBroadPhaseLayer == BP_MOVING;
    case MOVING:
        return true;
    default:
        return false;
    }
}


bool JPHUtil::ObjectLayerPairFilterImpl::ShouldCollide(ObjectLayer inObjectLayer1, ObjectLayer inObjectLayer2) const
{
    switch (inObjectLayer1)
    {
    case NON_MOVING:
        return inObjectLayer2 == MOVING;
    case MOVING:
        return true;
    default:
        return false;
    }
}


void JPHUtil::VehicleBodyActivationListener::OnBodyActivated(const BodyID& inBody, uint64 inBodyUserData)
{
    std::cout << "A body got activated" << std::endl;
}


void JPHUtil::VehicleBodyActivationListener::OnBodyDeactivated(const BodyID& inBody, uint64 inBodyUserData)
{
    std::cout << "A body went to sleep" << std::endl;
}


JPH::ValidateResult JPHUtil::VehicleContactListener::OnContactValidate(const Body& inBody1, const Body& inBody2, RVec3Arg inBaseOffset, const CollideShapeResult& inCollisionResult)
{
    return ValidateResult::AcceptAllContactsForThisBodyPair;
}


void JPHUtil::VehicleContactListener::OnContactAdded(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings)
{
}


void JPHUtil::VehicleContactListener::OnContactPersisted(const Body& inBody1, const Body& inBody2, const ContactManifold& inManifold, ContactSettings& ioSettings)
{
}


void JPHUtil::VehicleContactListener::OnContactRemoved(const SubShapeIDPair& inSubShapePair)
{
}