from spatialmath import SE3


#mtri tranform to desired position translation and rotation based in a1 to a2 vector
start = [0.4,0.4,0.4]
end  = [0.4,-0.4,0.4]
direction = SE3.OA([1,0,0],[0,1,0])

Tst = SE3(start)*direction
Ted = SE3(end)*direction