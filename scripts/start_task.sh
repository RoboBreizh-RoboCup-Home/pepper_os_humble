Help()
{
# Display Help
echo "start_task - starts the file robobreizh_manager.launch"
echo
echo "Usage: start_task (-l|-h) TASK_NAME [door]"
echo "options:"
echo " -l               Print all the task names"
echo " -h               Print this Help."
echo " TASK_NAME        Name of the task."
echo "                  Can be listed with the option -l"
echo
}

is_task(){
    local LIST=`echo $1 | tr " " " "`
    local VALUE=$2
    for x in $LIST
    do
        if [ "$x" = "$VALUE" ]
        then
            return 0
        fi
    done
    return 1
}

Task()
{
# Display every task argument
echo "List of possible task argument:"
echo "  receptionist"
echo "  find_my_mates"
echo "  restaurant"
echo "  gpsr"
echo "  carry_my_luggage"
echo "  store_groceries"
echo
}

while getopts "lhd:" option
do
    case "${option}" in
        l) # List every possible task
            Task
            exit;;
        h) # Display help
            Help
            exit;;
        \?) # Invalid option
            echo "Error: Invalid option"
            exit;;
    esac
done

task=$1
door=$2

# Verifies the task parameter
tasks="receptionist find_my_mates restaurant gpsr carry_my_luggage store_groceries"
is_task "$tasks" "$task"
if [ $? != 0 ]
then
    echo " no such option ${task}"
    Task
    exit;
fi

# Verifies the door parameter
if [ -z "$door" ]
then
    door=false
else
    if [[ "$door" != "door" ]]
    then
        echo " door parameter must be the string 'door'"
        Help
        exit;
    fi
    door="true"
fi

# start command task
echo "roslaunch manager_pepper robobreizh_manager.launch ${task}:=true door:=${door}"
roslaunch manager_pepper robobreizh_manager.launch $task:=true door:=$door
