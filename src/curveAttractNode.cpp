#include "curveAttractNode.h"
#include <algorithm>


double const M_PI_HALF = M_PI * 0.5;

MTypeId CurveAttractNode::id(0x03);
MObject CurveAttractNode::aInputCurve;
MObject CurveAttractNode::aDistance;
MObject CurveAttractNode::aFalloffRamp;
MObject CurveAttractNode::aUseNormal;
MObject CurveAttractNode::aSmoothingStep;
MObject CurveAttractNode::aSmoothingIterations;

const MString CurveAttractNode::kName("curveAttract");


MStatus CurveAttractNode::initialize() {
	MStatus status;
	MFnTypedAttribute fnTypedAttr;
	MFnNumericAttribute fnNumericAttr;

	aInputCurve = fnTypedAttr.create("inputCurve", "inputCurve", MFnData::kNurbsCurve);
	fnTypedAttr.setStorable(true);
	fnTypedAttr.setKeyable(false);
	fnTypedAttr.setReadable(false);
	fnTypedAttr.setWritable(true);
	fnTypedAttr.setCached(false);
	addAttribute(aInputCurve);

	aDistance = fnNumericAttr.create("distance", "distance", MFnNumericData::kFloat, 1);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setStorable(true);
	fnNumericAttr.setKeyable(true);
	fnNumericAttr.setReadable(true);
	fnNumericAttr.setWritable(true);
	addAttribute(aDistance);

	aFalloffRamp = MRampAttribute::createCurveRamp("falloff", "falloff");
    addAttribute(aFalloffRamp);

	aUseNormal = fnNumericAttr.create("useNormal", "useNormal", MFnNumericData::kFloat, 1);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setMax(1);
	fnNumericAttr.setStorable(true);
	fnNumericAttr.setKeyable(true);
	fnNumericAttr.setReadable(true);
	fnNumericAttr.setWritable(true);
	addAttribute(aUseNormal);

    aSmoothingStep = fnNumericAttr.create("smoothingStep", "smoothingStep", MFnNumericData::kFloat, 0.5);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setMax(1);
	fnNumericAttr.setStorable(true);
	fnNumericAttr.setKeyable(true);
	fnNumericAttr.setReadable(true);
	fnNumericAttr.setWritable(true);
	addAttribute(aSmoothingStep);

	aSmoothingIterations = fnNumericAttr.create("smoothingIterations", "smoothingIterations", MFnNumericData::kInt, 0);
	fnNumericAttr.setMin(0);
	fnNumericAttr.setStorable(true);
	fnNumericAttr.setKeyable(true);
	fnNumericAttr.setReadable(true);
	fnNumericAttr.setWritable(true);
	addAttribute(aSmoothingIterations);

	attributeAffects(aInputCurve, outputGeom);
	attributeAffects(aDistance, outputGeom);
	attributeAffects(aFalloffRamp, outputGeom);
	attributeAffects(aUseNormal, outputGeom);
	attributeAffects(aSmoothingStep, outputGeom);
	attributeAffects(aSmoothingIterations, outputGeom);

	status = MGlobal::executeCommandOnIdle("makePaintable -attrType multiFloat -sm deformer curveAttract weights");
    CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;
}


MStatus CurveAttractNode::initializeRamp(MObject& oParent, MObject& oValueRamp, int index, float position, float value, int interpolation)
{
    MStatus status;

    MPlug pValueRamp(oParent, oValueRamp);
    MPlug pValueRampElement = pValueRamp.elementByLogicalIndex(index, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MPlug pValueRampPosition = pValueRampElement.child(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = pValueRampPosition.setFloat(position);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MPlug pValueRampValue = pValueRampElement.child(1, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = pValueRampValue.setFloat(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MPlug pValueRampInterpolation = pValueRampElement.child(2);
    pValueRampInterpolation.setInt(interpolation);

    return MS::kSuccess;
}


void* CurveAttractNode::creator() {
	return new CurveAttractNode();
}


void CurveAttractNode::postConstructor()
{
    initializeRamp(thisMObject(), aFalloffRamp, 0, 0.0f, 0.0f, 2);
    initializeRamp(thisMObject(), aFalloffRamp, 1, 1.0f, 1.0f, 2);
}


CurveAttractNode::CurveAttractNode() {
}


CurveAttractNode::~CurveAttractNode() {
}


MStatus CurveAttractNode::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex) {
    MStatus status;

    MArrayDataHandle dInputs = block.inputValue(input, &status);
    status = dInputs.jumpToElement(multiIndex);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle dInput = dInputs.inputValue(&status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle dGeom = dInput.child(inputGeom);
    MObject oGeom = dGeom.asMesh();
    MMatrix matrix = dGeom.geometryTransformMatrix();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle dInputCurve = block.inputValue(aInputCurve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	MObject oInputCurve = dInputCurve.asNurbsCurve();

	float weight = block.inputValue(envelope).asFloat();
	if (weight == 0.0f || oInputCurve.isNull()) {
        return MS::kSuccess;
    }

    MDataHandle dDistance = block.inputValue(aDistance, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	float distance = dDistance.asFloat();

	MRampAttribute fnFalloffRamp(thisMObject(), aFalloffRamp, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

    MDataHandle dUseNormal = block.inputValue(aUseNormal, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	float useNormal = dUseNormal.asFloat();

	MDataHandle dSmoothingStep = block.inputValue(aSmoothingStep, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    float smoothingStep = dSmoothingStep.asFloat();

    MDataHandle dSmoothingIterations = block.inputValue(aSmoothingIterations, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    int smoothingIterations = dSmoothingIterations.asInt();

    int prevIndex;
    float length;
    float weightRamp;
    float weightVertex;
    double param;

    MIntArray vertices;
    MPointArray points;
    MPointArray pointsSmooth;

    std::map<int, int> indices;
    std::map<int, float> smoothing;
    std::map<int, std::unordered_set<int>> connectivity;

    int count = iter.count(&status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = iter.allPositions(points);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MItMeshVertex itGeom(oGeom, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MFnNurbsCurve fnInputCurve(oInputCurve, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // construct a map between vertex index and ordered index, this will most
    // likely be the same unless the deformer was created using components.
    for (int i = 0; i < count; i++) {
        int index = iter.index(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        indices[index] = i;
        iter.next();
    }

    // calculate the initial displacement of the vertices and store the vertex
    // connectivity used for smoothing operations, these smoothing operations
    // will work hardest on vertices that have a 90 degree angle between
    // normal and closest point.
    for (auto const& index: indices) {
        float smooth;
        MVector normal;
        MPoint point = points[index.second] * matrix;

        status = itGeom.setIndex(index.first, prevIndex);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        prevIndex = index.first;

        status = itGeom.getNormal(normal, MSpace::kWorld);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        MPoint pointOnCurve = fnInputCurve.closestPoint(point, &param, 0.0, MSpace::kWorld, &status);

        MVector vector = (MVector)pointOnCurve - (MVector)point;
        length = vector.length() - distance;
        length = std::min(1.0f, std::max(0.0f, 1.0f - (length / distance)));

        fnFalloffRamp.getValueAtPosition(length, weightRamp, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        weightVertex = weightValue(block, multiIndex, index.first);

        if (normal.angle(vector) > M_PI_HALF) {
            normal *= -1;
        }

        float dot = vector.normal() * normal;
        float multiplier = weight * weightRamp * weightVertex;

        if (multiplier > 0.00001f && smoothingIterations > 0) {
            fnFalloffRamp.getValueAtPosition(1.0f - std::abs(dot), smooth, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            smoothing[index.second] = smooth;

            status = itGeom.getConnectedVertices(vertices);
            CHECK_MSTATUS_AND_RETURN_IT(status);

            for (int i = 0; i < vertices.length(); i++) {
                if (indices.find(vertices[i]) != indices.end()) {
                    connectivity[index.second].insert(indices[vertices[i]]);
                }
            }
        }

        MQuaternion rotation = normal.rotateTo(vector);
        rotation = slerp(MQuaternion::identity, rotation, 1.0f - useNormal);
        normal *= rotation.asMatrix();

        point = MPoint((MVector)point + (normal.normal() * vector.length() * dot * multiplier));
        points.set(point * matrix.inverse(), index.second);
    }

    // smooth the points using the vertex connectivity.
    for (int i = 0; i < smoothingIterations; ++i)
    {
        pointsSmooth.copy(points);

        for (auto const& connect : connectivity)
        {
            int num = connect.second.size();
            float smooth = smoothing[connect.first] * smoothingStep;
            MVector point = (MVector)points[connect.first];
            MVector pointSmooth;

            for (auto const& index: connect.second)
            {
                pointSmooth += ((point / num) * (1 - smooth)) + ((points[index] / num) * smooth);
            }
            pointsSmooth.set(pointSmooth, connect.first);
        }
        points.copy(pointsSmooth);
    }

    iter.setAllPositions(points);

    return MS::kSuccess;
}