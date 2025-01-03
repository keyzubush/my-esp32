while read -rsn1 ui; do
    case "$ui" in
    $'\x1b')    # Handle ESC sequence.
        # Flush read. We account for sequences for Fx keys as
        # well. 6 should suffice far more then enough.
        read -rsn1 -t 0.05 tmp
        if [[ "$tmp" == "[" ]]; then
            read -rsn1 -t 0.05 tmp
            case "$tmp" in
            "A") curl http://192.168.1.190:5000/forward ;;
            "B") curl http://192.168.1.190:5000/backward ;;
            "C") curl http://192.168.1.190:5000/right ;;
            "D") curl http://192.168.1.190:5000/left ;;
            esac
        fi
        # Flush "stdin" with 0.1  sec timeout.
	read -rsn5 -t 0.5
        ;;
    # Other one byte (char) cases. Here only quit.
    q) break;;
    s) curl http://192.168.1.190:5000/stop ;;
    a) curl http://192.168.1.190:5000/leftstep ;;
    d) curl http://192.168.1.190:5000/rightstep ;;
    esac
done
