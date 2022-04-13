import { useState, useEffect } from "react";
import { Publisher } from "roslib-reactjs";
import { Button, Grid, Box, FormControlLabel, Switch } from "@mui/material";

interface ControlButtonProps {
    keyboardKey: string;
    callback: () => void;
}

function ControlButton(props: ControlButtonProps) {
    return (
        <Button onClick={() => props.callback()}>
            {props.keyboardKey}
        </Button>
    );
}

interface FetchKeyboardControllerProps {
    pubTopic: string;
    pubType: string;
    queueSize: 1;
}

interface Twist {
    linear: {
        x: number, // Set positive or negative meters/s to drive
        y: number,
        z: number
    },
    angular: {
        x: number,
        y: number,
        z: number // Set rads/s to turn
    }
}

const ZERO_TWIST: Twist = {
    linear: {
        x: 0, // Set positive or negative meters/s to drive
        y: 0,
        z: 0
    },
    angular: {
        x: 0,
        y: 0,
        z: 0 // Set rads/s to turn
    }
}

function FetchKeyboardController(props: FetchKeyboardControllerProps) {

    // TODO: If there are performance issues, consider taking the message Publisher part into separate component
    const [message, setMessage] = useState<Twist>(ZERO_TWIST);
    const [isHalt, setIsHalt] = useState<{value: boolean}>({value: false});

    const buttonHandler = () => {
      setIsHalt(current => {return {value: !current.value}});
    }

    function publishControl(linear?: number, angular?: number) {
        // If halted ignore all input
        if (isHalt.value) {
            linear = 0.0;
            angular = 0.0;
        }

        if (linear === undefined) {
            linear = message.linear.x;
        }
        if (angular === undefined) {
            angular = message.angular.z;
        }

        setMessage(
            {
                linear: {
                    x: linear, // Set positive or negative meters/s to drive
                    y: 0,
                    z: 0
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: angular // Set rads/s to turn
                }
            }
        );
    }

    function onKeyPress(event: KeyboardEvent) {
        switch (event.key) {
            case "w":
                publishControl(1.0, 0);
                break;
            case "s":
                publishControl(-1.0, 0);
                break;
            case "a":
                publishControl(0, 1.0);
                break;
            case "d":
                publishControl(0, -1.0);
                break;
            default:
                break;
        }
    }

    useEffect(() =>
    {
        window.addEventListener("keydown", onKeyPress);
        return () =>
        {
            window.removeEventListener("keydown", onKeyPress);
        }
    // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [isHalt])

    console.log("Post change: " + isHalt);

    return (
        <Grid container spacing={3} alignItems='center'>
            <Publisher
                name={props.pubTopic}
                type={props.pubType}
                queue_size={props.queueSize}
                message={message}
            />
            <Grid item xs={12}>
                <h2>Keyboard</h2>
            </Grid>
            <Box sx={{ width: 250 }}>
                <Grid container spacing={{ xs: 1, md: 2 }} columns={{ xs: 1, sm: 2, md: 3 }} width={10}>
                    <Grid item xs={1}/>
                    <Grid item xs={1}>
                        <ControlButton keyboardKey='w' callback={() => publishControl(1.0, 0.0)}/>
                    </Grid>
                    <Grid item xs={1}/>
                    <Grid item xs={1}>
                        <ControlButton keyboardKey='a' callback={() => publishControl(0.0, 1.0)}/>
                    </Grid>
                    <Grid item xs={1}>
                        <ControlButton keyboardKey='s' callback={() => publishControl(-1.0, 0.0)}/>
                    </Grid>
                    <Grid item xs={1}>
                        <ControlButton keyboardKey='d' callback={() => publishControl(0.0, -1.0)}/>
                    </Grid>
                </Grid>
            </Box>
            <Grid item xs={6}>
                <FormControlLabel control={<Switch onChange={buttonHandler} />} label={isHalt.value ? "E-STOP" : "Normal"} />
            </Grid>
        </Grid>
    );
}

export { FetchKeyboardController };
