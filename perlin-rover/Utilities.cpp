#include "Utilities.h"

#include <fstream>
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

void OBJUtil::OBJExporter::SaveToObj(const std::vector<Vertex>& vertices, 
    const std::vector<UV>& uvs, const std::vector<Normal>& normals, 
    const std::vector<Triangle>& triangles, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (const auto& vertex : vertices)
    {
        file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    for (const auto& uv : uvs)
    {
        file << "vt " << uv.u << " " << uv.v << "\n";
    }

    for (const auto& normal : normals)
    {
        file << "vn " << normal.nx << " " << normal.ny << " " << normal.nz << "\n";
    }

    for (const auto& triangle : triangles)
    {
        // OBJ uses 1-based indexing, so add 1 to each index
        file << "f "
            << triangle.v1 + 1 << "/" << triangle.uv1 + 1 << "/" << triangle.n1 + 1 << " "
            << triangle.v2 + 1 << "/" << triangle.uv2 + 1 << "/" << triangle.n2 + 1 << " "
            << triangle.v3 + 1 << "/" << triangle.uv3 + 1 << "/" << triangle.n3 + 1 << "\n";
    }


    file.close();
}