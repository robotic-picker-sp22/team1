import { Slider, Box, Grid, Input, Button } from '@mui/material';
import { useState, ChangeEvent, SyntheticEvent } from 'react';
import { ServiceCaller } from 'roslib-reactjs';
import { ShowText } from './ShowText.tsx';
import { Subscriber, useMsg } from 'roslib-reactjs';

interface ROSSliderProps {
    name: string;
    body_part: string;
    joint: string;
    startingValue?: number;
    statusTopic?: string;
    min: number;
    max: number;
    numSteps?: number;
    serviceName?: string;
    serviceType?: string;
    width?: number;
    onChange?: (value: number) => void;
}

function FirstTimeSubscriberCallback(props: {
    setValue: (value: number) => void;
    setBool: (value: boolean) => void;
}) {
  const msg = useMsg();
  if (msg === null) {
    return <></>;
  }
  const value = msg.data;
  props.setValue(value);
  props.setBool(true);
  return <></>;
}

function ROSSlider(props: ROSSliderProps) {
    const max_val = typeof props.max === 'number' ? props.max : 100;
    const min_val = typeof props.min === 'number' ? props.min : 0;
    const width = typeof props.width === 'number' ? props.width : 750;
    const num_steps = typeof props.numSteps === 'number' ? props.numSteps : 100;
    const startingValue = typeof props.startingValue === 'number' ? props.startingValue : min_val;
    const serviceName = typeof props.serviceName === 'string' ? props.serviceName : '/web_teleop/set_joint_value';
    const serviceType = typeof props.serviceType === 'string' ? props.serviceType : 'web_teleop/SetJointValue';
    const onChange = typeof props.onChange === 'function' ? props.onChange : (value: number) => {};
    const step = (max_val - min_val) / num_steps;

    const [value, setValue] = useState<number>(
      startingValue
    );
    const [sendRequest, setSendRequest] = useState<{state: boolean}>({state: false});
    const [jointValueRequest, setJointValueRequest] = useState<{value: number, body_part: string, joint: string}>({
      value: startingValue,
      body_part: props.body_part,
      joint: props.joint
    });
    const [readValueFromSub, setReadValueFromSub] = useState<Boolean>(false);

    const clipValue = (value: number) => {
      if (value < min_val) {
        return min_val;
      }
      if (value > max_val) {
        return max_val;
      }
      return value;
    }

    const handleValueChange = (newValue: number) => {
      newValue = clipValue(newValue);
      onChange(newValue);
      setValue(newValue);
    }

    const handleSliderChange = (event: Event | SyntheticEvent, newValue: number | number[]) => {
      handleValueChange(newValue as number);
    };

    const handleInputChange = (event: ChangeEvent<HTMLInputElement>) => {
      handleValueChange(event.target.value === '' ? startingValue : Number(event.target.value));
    };

    const onButtonClick = () => {
      setJointValueRequest({value: value, body_part: props.body_part, joint: props.joint});
      setSendRequest({state: true});
    }

    return (
      <Box sx={{ width: width }}>
        <ServiceCaller
          name={serviceName}
          type={serviceType}
          request={jointValueRequest}
          callback={(data) => {setSendRequest({state: false})}}
          failedCallback={(msg) => {setSendRequest({state: false}); console.warn("Callback Failed: " + msg)}}
          toggler={sendRequest.state}
        />
        <Grid container spacing={2} alignItems="center">
          <Grid item xs={3}>
            {props.name}
          </Grid>
          <Grid item xs>
            <Slider
              key={`slider-${value}`}
              defaultValue={value}
              onChangeCommitted={handleSliderChange}
              aria-labelledby="input-slider"
              max={max_val}
              min={min_val}
              step={step}
            />
          </Grid>
          <Grid item xs={2}>
            <Input
              value={value}
              size="small"
              onChange={handleInputChange}
              inputProps={{
                step: step * 10,
                min: min_val,
                max: max_val,
                type: 'number',
                'aria-labelledby': 'input-slider',
              }}
            />
          </Grid>
          {
            serviceType !== "" && serviceName !== "" ?
            <Grid item xs={1}>
              <Button onClick={onButtonClick}>Send</Button>
            </Grid> :
            <></>
          }
          {
            props.statusTopic ?
            <Grid item xs={2}>
              <Subscriber
                name={props.statusTopic}
                type="std_msgs/Float64"
                rate={10.0}
                queue_size={10}
              >
                {!readValueFromSub ? <FirstTimeSubscriberCallback setValue={handleValueChange} setBool={setReadValueFromSub} /> : <></>}
                <ShowText name="Robot"/>
              </Subscriber>
            </Grid> : <></>
          }
        </Grid>
      </Box>
    );
}

export { ROSSlider };