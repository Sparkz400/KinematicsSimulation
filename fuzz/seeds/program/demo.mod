MODULE m
 CONST robtarget p1 := [[0,1000,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
 VAR speeddata vfast := [250, 500, 5000, 1000];
 PROC main()
  MoveL [[-1000,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, z10, tool0;
  MoveC p1, [[1000,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], vfast, fine, tool0; ! half
  MoveJ [[0,0,0],[0.7071068,0,0,0.7071068],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], vmax, fine, tool0;
 ENDPROC
ENDMODULE
