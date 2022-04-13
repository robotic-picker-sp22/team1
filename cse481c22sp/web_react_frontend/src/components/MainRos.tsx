import { RosConnect } from 'roslib-reactjs';
import { ROSSlider } from './slider/ROSSlider.tsx';
import { FetchKeyboardController } from './keyboard_teleop/KeyboardTeleop.tsx';
import { Grid } from '@mui/material';
import { GripperSlider } from './slider/GripperSlider.tsx';

function degs_to_rad(val: number) {
  return Math.PI * val / 180.0;
}

function ConnectedPanel() {

  return (
    <RosConnect
      url="ws://localhost:9090"
      autoconnect
      timeout={1000}
    >
      <Grid container spacing={3}>
        <Grid item xs={12}>
          <ROSSlider name="Head/Tilt" min={-Math.PI / 4} max={Math.PI / 2} body_part="head" joint="tilt" statusTopic="/joint_state_republisher/head_tilt_joint"/>
          <ROSSlider name="Head/Pan" min={-Math.PI / 2} max={Math.PI / 2} body_part="head" joint="pan" statusTopic="/joint_state_republisher/head_pan_joint"/>
          <ROSSlider name="Torso" min={0.0} max={0.4} body_part="torso" joint="" statusTopic="/joint_state_republisher/torso_lift_joint"/>
          <ROSSlider name="Arm/Shoulder Pan" min={-degs_to_rad(90)} max={degs_to_rad(92)} body_part="arm" joint="shoulder_pan" statusTopic="/joint_state_republisher/shoulder_pan_joint"/>
          <ROSSlider name="Arm/Shoulder Lift" min={-degs_to_rad(70)} max={degs_to_rad(87)} body_part="arm" joint="shoulder_lift" statusTopic="/joint_state_republisher/shoulder_lift_joint"/>
          <ROSSlider name="Arm/Elbow Flex" min={-degs_to_rad(129)} max={degs_to_rad(129)} body_part="arm" joint="elbow_flex" statusTopic="/joint_state_republisher/elbow_flex_joint" />
          <ROSSlider name="Arm/Wrist Flex" min={-degs_to_rad(125)} max={degs_to_rad(125)} body_part="arm" joint="wrist_flex" statusTopic="/joint_state_republisher/wrist_flex_joint"/>
          <GripperSlider/>
        </Grid>
        <Grid item xs={12}>
          <FetchKeyboardController
            pubTopic="/cmd_vel"
            pubType="geometry_msgs/Twist"
          />
        </Grid>
        <Grid item xs={12}>
        <div id="camera">
          <img src="//localhost:8234/stream?topic=/head_camera/rgb/image_raw" alt=""></img>
        </div>
        </Grid>
      </Grid>
    </RosConnect>
  );
}

export default ConnectedPanel;
