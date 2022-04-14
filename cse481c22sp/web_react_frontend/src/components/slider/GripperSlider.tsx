import { ROSSlider } from "./ROSSlider.tsx";
import { Grid, FormControlLabel, Switch } from '@mui/material';
import { useState } from "react";
import { ServiceCaller } from 'roslib-reactjs';

function GripperSlider() {
    const body_part = "gripper"

    const [curGripperForce, setCurGripperForce] = useState<number>(100);

    const [isOn, setIsOn] = useState<Boolean>(false);
    const [sendRequest, setSendRequest] = useState<{state: boolean}>({state: false});
    const [jointValueRequest, setJointValueRequest] = useState<{value: number, body_part: string, joint: string}>({
        value: isOn ? curGripperForce : -1,
        body_part: body_part,
        joint: ""
      });

      function buttonHandler() {
        setJointValueRequest({value: isOn ? 0 : curGripperForce, body_part: body_part, joint: isOn ? "close" : "open"});
        setSendRequest({state: true});
        setIsOn(current => {return !current});
    }

    const serviceName = "/web_teleop/set_joint_value";
    const serviceType = "web_teleop/SetJointValue";


    return <Grid container spacing={2} alignItems="center" width={800}>
        <ServiceCaller
          name={serviceName}
          type={serviceType}
          request={jointValueRequest}
          callback={(data) => {setSendRequest({state: false}); console.log("Callback Succeeded: " + data)}}
          failedCallback={(msg) => {setSendRequest({state: false}); console.warn("Callback Failed: " + msg)}}
          toggler={sendRequest.state}
        />
        <Grid item xs={10}>
            <ROSSlider
                name="Gripper"
                min={35}
                max={100}
                numSteps={65}
                serviceName={""}
                serviceType={""}
                body_part="gripper"
                joint=""
                width={625}
                onChange={setCurGripperForce}
            />
        </Grid>
        <Grid item xs={2}>
            <FormControlLabel control={<Switch onChange={buttonHandler} defaultValue={isOn ? 1 : 0} />} label="" />
        </Grid>
    </Grid>
}

export { GripperSlider };