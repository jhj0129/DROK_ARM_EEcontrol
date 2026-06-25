# gcode_interpreter/msg/GcodeCommand.msg
# G-code 한 줄의 명령 전체를 표현

# G-code의 주요 명령 코드 (예: "G1", "M3", "G90")
string command_code

# 해당 명령에 포함된 모든 파라미터들의 배열
Param[] params
