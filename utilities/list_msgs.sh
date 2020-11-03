#! /bin/sh
#
# list_msgs.sh

listmsg() {
    echo "### $1"
    rostopic info "$1"
}

echo "Generated from \`/utilities/list_msgs.sh\`"
echo ""
listmsg /SVEA/3D_car
listmsg /SVEA/lli/ctrl_actuated
listmsg /SVEA/lli/ctrl_request
listmsg /SVEA/lli/remote
listmsg /SVEA/past_path
listmsg /SVEA/path_plan
listmsg /SVEA/state
listmsg /SVEA/target
listmsg /SVEA/vis_pose
listmsg /SVEA/viz_edges
listmsg /SVEA/viz_lidar_points
listmsg /SVEA/viz_lidar_rays
listmsg /map
listmsg /map_metadata
listmsg /rosout
listmsg /rosout_agg
listmsg /scan
