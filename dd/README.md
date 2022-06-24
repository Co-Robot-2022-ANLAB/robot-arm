
roscore

roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=192.168.0.39(아이피 바뀔수도 있음)

ping (ip) 아이피 활성화 확인

roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch (무브잇)

roslaunch ur3_moveit_config moveit_rviz.launch config:=true (rivz)

rosrun ur3_moveit find_position.py 위치찾기

rosrun ur3_moveit ur3_demo.py 잡고 움직이기

rosrun ur3_moveit ur3_demo_2.py 연습하고 있는거(나름됨)

rosrun ur3_moveit ur3_demo_3.py 쿼터니언 고정후 움직이기


그리퍼 노드 활성화

rosrun ur_rg2_gripper gripper_node.py

rostopic echo /gripper/command

--------------좌표노가다-----------
1에서
position: 
    x: 0.271754835847
    y: -0.46357723769
    z: 0.173816733563
  orientation: 
    x: 0.44686850379
    y: 0.564310253224
    z: -0.427157139455
    w: 0.54717388154


터미널해서
position: 
    x: 0.50915548519
    y: -0.184583470808
    z: 0.107037743379
  orientation: 
    x: 0.481765970983
    y: 0.537068621189
    z: -0.389525244439
    w: 0.572476138613



2로 이동
position: 
    x: 0.512494368266
    y: 0.092306970458
    z: 0.273852643888
  orientation: 
    x: 0.478536416702
    y: 0.537743450154
    z: -0.394881116221
    w: 0.570879833028


다운 포지션
position: 
    x: 0.542257993123
    y: 0.110226735644
    z: 0.0640024528185
  orientation: 
    x: -0.707065065825
    y: -0.707022585585
    z: 0.00943507492687
    w: 0.00943586368007

손목꺽기
 x: 0.542257993123
    y: 0.110226735644
    z: 0.0640024528185
오리엔탈
x: -0.00639343787702
    y: 0.706865091323
    z: -0.0113791222389
    w: 0.707227956318


베이스 30도 회전
position: 
    x: 0.415955101837
    y: 0.366181673596
    z: 0.0652537202911
  orientation: 
    x: -0.192572044226
    y: 0.674844113173
    z: 0.168829104853
    w: 0.692096932556


-----------------------------------

https://roomedia.tistory.com/entry/41%EC%9D%BC%EC%B0%A8-%EB%A7%A4%EB%8B%88%ED%93%B0%EB%A0%88%EC%9D%B4%EC%85%98-%EC%86%8C%EA%B0%9C-%EB%B0%8F-URDF-%EC%9E%91%EC%84%B1%EB%B2%95 이게 urdf만드는거?
https://mrlacquer.tistory.com/7 자크로파일 함 만들어보기

6/22일

뭐였지 무브잇 컨피그 들어가서 컨트롤러 얌파일 바꿈 안되면 다시 찾아서 원본으로 바꿔놓기

6/23일

ur3.urdf.xacro 파일보면 주석으로 뭐 바꿨는지 적어놈


