#include <map>
#include <unordered_set>
#include <maya/MPxNode.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MTypes.h>
#include <maya/MStatus.h>
#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MRampAttribute.h>


class CurveAttractNode : public MPxDeformerNode {
public:
	CurveAttractNode();
	virtual ~CurveAttractNode();
	virtual void postConstructor();
	static void* creator();

	static MStatus initialize();
	static MStatus initializeRamp(MObject& oParent, MObject& oColorRamp, int index, float position, float value, int interpolation);

	virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);

	static MTypeId id;
	static const MString kName;
	static MObject aInputCurve;
	static MObject aDistance;
	static MObject aFalloffRamp;
	static MObject aParameterRamp;
	static MObject aUseNormal;
	static MObject aSmoothingStep;
	static MObject aSmoothingIterations;
};
