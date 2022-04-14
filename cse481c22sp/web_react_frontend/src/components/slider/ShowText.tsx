import { useMsg } from 'roslib-reactjs';

function round(value, decimals) {
    const multiplier = Math.pow(10, decimals);
    return Math.round((value + Number.EPSILON) * multiplier) / multiplier;
}

interface ShowTextProps {
    name: string;
}

function ShowText(props: ShowTextProps) {
    const msg = useMsg();
    if (msg == null) {
        return <div>Waiting for message...</div>;
    }

    return (
        <div>
            {props.name}: {round(msg.data, 3)}
        </div>
    );
}

export { ShowText };