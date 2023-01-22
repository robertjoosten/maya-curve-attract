#include <maya/MFnPlugin.h>

#include "curveAttractNode.h"


MStatus initializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj, "Robert Joosten", "1.0", "any");

	status = plugin.registerNode(
		CurveAttractNode::kName,
		CurveAttractNode::id,
		CurveAttractNode::creator,
		CurveAttractNode::initialize,
		MPxNode::kDeformerNode
	);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;
}

MStatus uninitializePlugin(MObject obj) {
	MStatus status;
	MFnPlugin plugin(obj);

	status = plugin.deregisterNode(CurveAttractNode::id);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;
}