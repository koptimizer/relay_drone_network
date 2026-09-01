\ Model Disaster_Drone_Routing
\ LP format - for model browsing. Use MPS format to capture full model detail.
\ Signature: 0xd41201d4f30c0f7b
Minimize
  0.001 y[R1,1] + 0.001 y[R1,2] + 0.001 y[R1,3] + 0.001 y[R1,4]
   + 0.001 y[R1,5] + 0.001 y[R1,6] + 0.001 y[R1,7] + 0.001 y[R1,8]
   + 0.001 y[R1,9] + 0.001 y[R1,10] + 0.001 y[R1,11] + 0.001 y[R1,12]
   + 0.001 y[R1,13] + 0.001 y[R1,14] + 0.001 y[R1,15] + 0.001 y[R2,1]
   + 0.001 y[R2,2] + 0.001 y[R2,3] + 0.001 y[R2,4] + 0.001 y[R2,5]
   + 0.001 y[R2,6] + 0.001 y[R2,7] + 0.001 y[R2,8] + 0.001 y[R2,9]
   + 0.001 y[R2,10] + 0.001 y[R2,11] + 0.001 y[R2,12] + 0.001 y[R2,13]
   + 0.001 y[R2,14] + 0.001 y[R2,15] + C_max
Subject To
 makespan_K1: - C_k[K1] + C_max >= 0
 completion_time_K1_1: - z[K1,1] + C_k[K1] >= 0
 completion_time_K1_2: - 2 z[K1,2] + C_k[K1] >= 0
 completion_time_K1_3: - 3 z[K1,3] + C_k[K1] >= 0
 completion_time_K1_4: - 4 z[K1,4] + C_k[K1] >= 0
 completion_time_K1_5: - 5 z[K1,5] + C_k[K1] >= 0
 completion_time_K1_6: - 6 z[K1,6] + C_k[K1] >= 0
 completion_time_K1_7: - 7 z[K1,7] + C_k[K1] >= 0
 completion_time_K1_8: - 8 z[K1,8] + C_k[K1] >= 0
 completion_time_K1_9: - 9 z[K1,9] + C_k[K1] >= 0
 completion_time_K1_10: - 10 z[K1,10] + C_k[K1] >= 0
 completion_time_K1_11: - 11 z[K1,11] + C_k[K1] >= 0
 completion_time_K1_12: - 12 z[K1,12] + C_k[K1] >= 0
 completion_time_K1_13: - 13 z[K1,13] + C_k[K1] >= 0
 completion_time_K1_14: - 14 z[K1,14] + C_k[K1] >= 0
 completion_time_K1_15: - 15 z[K1,15] + C_k[K1] >= 0
 makespan_K2: - C_k[K2] + C_max >= 0
 completion_time_K2_1: - z[K2,1] + C_k[K2] >= 0
 completion_time_K2_2: - 2 z[K2,2] + C_k[K2] >= 0
 completion_time_K2_3: - 3 z[K2,3] + C_k[K2] >= 0
 completion_time_K2_4: - 4 z[K2,4] + C_k[K2] >= 0
 completion_time_K2_5: - 5 z[K2,5] + C_k[K2] >= 0
 completion_time_K2_6: - 6 z[K2,6] + C_k[K2] >= 0
 completion_time_K2_7: - 7 z[K2,7] + C_k[K2] >= 0
 completion_time_K2_8: - 8 z[K2,8] + C_k[K2] >= 0
 completion_time_K2_9: - 9 z[K2,9] + C_k[K2] >= 0
 completion_time_K2_10: - 10 z[K2,10] + C_k[K2] >= 0
 completion_time_K2_11: - 11 z[K2,11] + C_k[K2] >= 0
 completion_time_K2_12: - 12 z[K2,12] + C_k[K2] >= 0
 completion_time_K2_13: - 13 z[K2,13] + C_k[K2] >= 0
 completion_time_K2_14: - 14 z[K2,14] + C_k[K2] >= 0
 completion_time_K2_15: - 15 z[K2,15] + C_k[K2] >= 0
 start_kK1: x[0,C1,K1,1] + x[0,C2,K1,1] + x[0,R1,K1,1] + x[0,R2,K1,1]
   + v[0,K1,1] = 1
 no_start_C1_K1: v[C1,K1,1] = 0
 no_start_C2_K1: v[C2,K1,1] = 0
 no_start_R1_K1: v[R1,K1,1] = 0
 no_start_R2_K1: v[R2,K1,1] = 0
 start_kK2: x[0,C1,K2,1] + x[0,C2,K2,1] + x[0,R1,K2,1] + x[0,R2,K2,1]
   + v[0,K2,1] = 1
 no_start_C1_K2: v[C1,K2,1] = 0
 no_start_C2_K2: v[C2,K2,1] = 0
 no_start_R1_K2: v[R1,K2,1] = 0
 no_start_R2_K2: v[R2,K2,1] = 0
 flow_balance_0_K1_2: x[0,C1,K1,2] + x[0,C2,K1,2] + x[0,R1,K1,2]
   + x[0,R2,K1,2] - v[0,K1,1] + v[0,K1,2] = 0
 flow_balance_0_K1_3: x[0,C1,K1,3] + x[0,C2,K1,3] + x[0,R1,K1,3]
   + x[0,R2,K1,3] - x[R1,0,K1,1] - v[0,K1,2] + v[0,K1,3] = 0
 flow_balance_0_K1_4: x[0,C1,K1,4] + x[0,C2,K1,4] + x[0,R1,K1,4]
   + x[0,R2,K1,4] - x[C2,0,K1,1] - x[R1,0,K1,2] - x[R2,0,K1,1] - v[0,K1,3]
   + v[0,K1,4] = 0
 flow_balance_0_K1_5: x[0,C1,K1,5] + x[0,C2,K1,5] + x[0,R1,K1,5]
   + x[0,R2,K1,5] - x[C1,0,K1,1] - x[C2,0,K1,2] - x[R1,0,K1,3]
   - x[R2,0,K1,2] - v[0,K1,4] + v[0,K1,5] = 0
 flow_balance_0_K1_6: x[0,C1,K1,6] + x[0,C2,K1,6] + x[0,R1,K1,6]
   + x[0,R2,K1,6] - x[C1,0,K1,2] - x[C2,0,K1,3] - x[R1,0,K1,4]
   - x[R2,0,K1,3] - v[0,K1,5] + v[0,K1,6] = 0
 flow_balance_0_K1_7: x[0,C1,K1,7] + x[0,C2,K1,7] + x[0,R1,K1,7]
   + x[0,R2,K1,7] - x[C1,0,K1,3] - x[C2,0,K1,4] - x[R1,0,K1,5]
   - x[R2,0,K1,4] - v[0,K1,6] + v[0,K1,7] = 0
 flow_balance_0_K1_8: x[0,C1,K1,8] + x[0,C2,K1,8] + x[0,R1,K1,8]
   + x[0,R2,K1,8] - x[C1,0,K1,4] - x[C2,0,K1,5] - x[R1,0,K1,6]
   - x[R2,0,K1,5] - v[0,K1,7] + v[0,K1,8] = 0
 flow_balance_0_K1_9: x[0,C1,K1,9] + x[0,C2,K1,9] + x[0,R1,K1,9]
   + x[0,R2,K1,9] - x[C1,0,K1,5] - x[C2,0,K1,6] - x[R1,0,K1,7]
   - x[R2,0,K1,6] - v[0,K1,8] + v[0,K1,9] = 0
 flow_balance_0_K1_10: x[0,C1,K1,10] + x[0,C2,K1,10] + x[0,R1,K1,10]
   + x[0,R2,K1,10] - x[C1,0,K1,6] - x[C2,0,K1,7] - x[R1,0,K1,8]
   - x[R2,0,K1,7] - v[0,K1,9] + v[0,K1,10] = 0
 flow_balance_0_K1_11: x[0,C1,K1,11] + x[0,C2,K1,11] + x[0,R1,K1,11]
   + x[0,R2,K1,11] - x[C1,0,K1,7] - x[C2,0,K1,8] - x[R1,0,K1,9]
   - x[R2,0,K1,8] - v[0,K1,10] + v[0,K1,11] = 0
 flow_balance_0_K1_12: x[0,C1,K1,12] + x[0,C2,K1,12] + x[0,R1,K1,12]
   + x[0,R2,K1,12] - x[C1,0,K1,8] - x[C2,0,K1,9] - x[R1,0,K1,10]
   - x[R2,0,K1,9] - v[0,K1,11] + v[0,K1,12] = 0
 flow_balance_0_K1_13: x[0,C1,K1,13] + x[0,C2,K1,13] + x[0,R1,K1,13]
   + x[0,R2,K1,13] - x[C1,0,K1,9] - x[C2,0,K1,10] - x[R1,0,K1,11]
   - x[R2,0,K1,10] - v[0,K1,12] + v[0,K1,13] = 0
 flow_balance_0_K1_14: x[0,C1,K1,14] + x[0,C2,K1,14] + x[0,R1,K1,14]
   + x[0,R2,K1,14] - x[C1,0,K1,10] - x[C2,0,K1,11] - x[R1,0,K1,12]
   - x[R2,0,K1,11] - v[0,K1,13] + v[0,K1,14] = 0
 flow_balance_0_K1_15: x[0,C1,K1,15] + x[0,C2,K1,15] + x[0,R1,K1,15]
   + x[0,R2,K1,15] - x[C1,0,K1,11] - x[C2,0,K1,12] - x[R1,0,K1,13]
   - x[R2,0,K1,12] - v[0,K1,14] + v[0,K1,15] = 0
 flow_balance_C1_K1_2: x[C1,0,K1,2] + x[C1,C2,K1,2] + x[C1,R1,K1,2]
   + x[C1,R2,K1,2] - v[C1,K1,1] + v[C1,K1,2] = 0
 flow_balance_C1_K1_3: x[C1,0,K1,3] + x[C1,C2,K1,3] + x[C1,R1,K1,3]
   + x[C1,R2,K1,3] - v[C1,K1,2] + v[C1,K1,3] = 0
 flow_balance_C1_K1_4: x[C1,0,K1,4] + x[C1,C2,K1,4] + x[C1,R1,K1,4]
   + x[C1,R2,K1,4] - x[R2,C1,K1,1] - v[C1,K1,3] + v[C1,K1,4] = 0
 flow_balance_C1_K1_5: - x[0,C1,K1,1] + x[C1,0,K1,5] + x[C1,C2,K1,5]
   + x[C1,R1,K1,5] + x[C1,R2,K1,5] - x[C2,C1,K1,1] - x[R2,C1,K1,2]
   - v[C1,K1,4] + v[C1,K1,5] = 0
 flow_balance_C1_K1_6: - x[0,C1,K1,2] + x[C1,0,K1,6] + x[C1,C2,K1,6]
   + x[C1,R1,K1,6] + x[C1,R2,K1,6] - x[C2,C1,K1,2] - x[R1,C1,K1,1]
   - x[R2,C1,K1,3] - v[C1,K1,5] + v[C1,K1,6] = 0
 flow_balance_C1_K1_7: - x[0,C1,K1,3] + x[C1,0,K1,7] + x[C1,C2,K1,7]
   + x[C1,R1,K1,7] + x[C1,R2,K1,7] - x[C2,C1,K1,3] - x[R1,C1,K1,2]
   - x[R2,C1,K1,4] - v[C1,K1,6] + v[C1,K1,7] = 0
 flow_balance_C1_K1_8: - x[0,C1,K1,4] + x[C1,0,K1,8] + x[C1,C2,K1,8]
   + x[C1,R1,K1,8] + x[C1,R2,K1,8] - x[C2,C1,K1,4] - x[R1,C1,K1,3]
   - x[R2,C1,K1,5] - v[C1,K1,7] + v[C1,K1,8] = 0
 flow_balance_C1_K1_9: - x[0,C1,K1,5] + x[C1,0,K1,9] + x[C1,C2,K1,9]
   + x[C1,R1,K1,9] + x[C1,R2,K1,9] - x[C2,C1,K1,5] - x[R1,C1,K1,4]
   - x[R2,C1,K1,6] - v[C1,K1,8] + v[C1,K1,9] = 0
 flow_balance_C1_K1_10: - x[0,C1,K1,6] + x[C1,0,K1,10] + x[C1,C2,K1,10]
   + x[C1,R1,K1,10] + x[C1,R2,K1,10] - x[C2,C1,K1,6] - x[R1,C1,K1,5]
   - x[R2,C1,K1,7] - v[C1,K1,9] + v[C1,K1,10] = 0
 flow_balance_C1_K1_11: - x[0,C1,K1,7] + x[C1,0,K1,11] + x[C1,C2,K1,11]
   + x[C1,R1,K1,11] + x[C1,R2,K1,11] - x[C2,C1,K1,7] - x[R1,C1,K1,6]
   - x[R2,C1,K1,8] - v[C1,K1,10] + v[C1,K1,11] = 0
 flow_balance_C1_K1_12: - x[0,C1,K1,8] + x[C1,0,K1,12] + x[C1,C2,K1,12]
   + x[C1,R1,K1,12] + x[C1,R2,K1,12] - x[C2,C1,K1,8] - x[R1,C1,K1,7]
   - x[R2,C1,K1,9] - v[C1,K1,11] + v[C1,K1,12] = 0
 flow_balance_C1_K1_13: - x[0,C1,K1,9] + x[C1,0,K1,13] + x[C1,C2,K1,13]
   + x[C1,R1,K1,13] + x[C1,R2,K1,13] - x[C2,C1,K1,9] - x[R1,C1,K1,8]
   - x[R2,C1,K1,10] - v[C1,K1,12] + v[C1,K1,13] = 0
 flow_balance_C1_K1_14: - x[0,C1,K1,10] + x[C1,0,K1,14] + x[C1,C2,K1,14]
   + x[C1,R1,K1,14] + x[C1,R2,K1,14] - x[C2,C1,K1,10] - x[R1,C1,K1,9]
   - x[R2,C1,K1,11] - v[C1,K1,13] + v[C1,K1,14] = 0
 flow_balance_C1_K1_15: - x[0,C1,K1,11] + x[C1,0,K1,15] + x[C1,C2,K1,15]
   + x[C1,R1,K1,15] + x[C1,R2,K1,15] - x[C2,C1,K1,11] - x[R1,C1,K1,10]
   - x[R2,C1,K1,12] - v[C1,K1,14] + v[C1,K1,15] = 0
 flow_balance_C2_K1_2: x[C2,0,K1,2] + x[C2,C1,K1,2] + x[C2,R1,K1,2]
   + x[C2,R2,K1,2] - v[C2,K1,1] + v[C2,K1,2] = 0
 flow_balance_C2_K1_3: x[C2,0,K1,3] + x[C2,C1,K1,3] + x[C2,R1,K1,3]
   + x[C2,R2,K1,3] - v[C2,K1,2] + v[C2,K1,3] = 0
 flow_balance_C2_K1_4: - x[0,C2,K1,1] + x[C2,0,K1,4] + x[C2,C1,K1,4]
   + x[C2,R1,K1,4] + x[C2,R2,K1,4] - v[C2,K1,3] + v[C2,K1,4] = 0
 flow_balance_C2_K1_5: - x[0,C2,K1,2] - x[C1,C2,K1,1] + x[C2,0,K1,5]
   + x[C2,C1,K1,5] + x[C2,R1,K1,5] + x[C2,R2,K1,5] - x[R2,C2,K1,1]
   - v[C2,K1,4] + v[C2,K1,5] = 0
 flow_balance_C2_K1_6: - x[0,C2,K1,3] - x[C1,C2,K1,2] + x[C2,0,K1,6]
   + x[C2,C1,K1,6] + x[C2,R1,K1,6] + x[C2,R2,K1,6] - x[R1,C2,K1,1]
   - x[R2,C2,K1,2] - v[C2,K1,5] + v[C2,K1,6] = 0
 flow_balance_C2_K1_7: - x[0,C2,K1,4] - x[C1,C2,K1,3] + x[C2,0,K1,7]
   + x[C2,C1,K1,7] + x[C2,R1,K1,7] + x[C2,R2,K1,7] - x[R1,C2,K1,2]
   - x[R2,C2,K1,3] - v[C2,K1,6] + v[C2,K1,7] = 0
 flow_balance_C2_K1_8: - x[0,C2,K1,5] - x[C1,C2,K1,4] + x[C2,0,K1,8]
   + x[C2,C1,K1,8] + x[C2,R1,K1,8] + x[C2,R2,K1,8] - x[R1,C2,K1,3]
   - x[R2,C2,K1,4] - v[C2,K1,7] + v[C2,K1,8] = 0
 flow_balance_C2_K1_9: - x[0,C2,K1,6] - x[C1,C2,K1,5] + x[C2,0,K1,9]
   + x[C2,C1,K1,9] + x[C2,R1,K1,9] + x[C2,R2,K1,9] - x[R1,C2,K1,4]
   - x[R2,C2,K1,5] - v[C2,K1,8] + v[C2,K1,9] = 0
 flow_balance_C2_K1_10: - x[0,C2,K1,7] - x[C1,C2,K1,6] + x[C2,0,K1,10]
   + x[C2,C1,K1,10] + x[C2,R1,K1,10] + x[C2,R2,K1,10] - x[R1,C2,K1,5]
   - x[R2,C2,K1,6] - v[C2,K1,9] + v[C2,K1,10] = 0
 flow_balance_C2_K1_11: - x[0,C2,K1,8] - x[C1,C2,K1,7] + x[C2,0,K1,11]
   + x[C2,C1,K1,11] + x[C2,R1,K1,11] + x[C2,R2,K1,11] - x[R1,C2,K1,6]
   - x[R2,C2,K1,7] - v[C2,K1,10] + v[C2,K1,11] = 0
 flow_balance_C2_K1_12: - x[0,C2,K1,9] - x[C1,C2,K1,8] + x[C2,0,K1,12]
   + x[C2,C1,K1,12] + x[C2,R1,K1,12] + x[C2,R2,K1,12] - x[R1,C2,K1,7]
   - x[R2,C2,K1,8] - v[C2,K1,11] + v[C2,K1,12] = 0
 flow_balance_C2_K1_13: - x[0,C2,K1,10] - x[C1,C2,K1,9] + x[C2,0,K1,13]
   + x[C2,C1,K1,13] + x[C2,R1,K1,13] + x[C2,R2,K1,13] - x[R1,C2,K1,8]
   - x[R2,C2,K1,9] - v[C2,K1,12] + v[C2,K1,13] = 0
 flow_balance_C2_K1_14: - x[0,C2,K1,11] - x[C1,C2,K1,10] + x[C2,0,K1,14]
   + x[C2,C1,K1,14] + x[C2,R1,K1,14] + x[C2,R2,K1,14] - x[R1,C2,K1,9]
   - x[R2,C2,K1,10] - v[C2,K1,13] + v[C2,K1,14] = 0
 flow_balance_C2_K1_15: - x[0,C2,K1,12] - x[C1,C2,K1,11] + x[C2,0,K1,15]
   + x[C2,C1,K1,15] + x[C2,R1,K1,15] + x[C2,R2,K1,15] - x[R1,C2,K1,10]
   - x[R2,C2,K1,11] - v[C2,K1,14] + v[C2,K1,15] = 0
 flow_balance_R1_K1_2: x[R1,0,K1,2] + x[R1,C1,K1,2] + x[R1,C2,K1,2]
   + x[R1,R2,K1,2] - v[R1,K1,1] + v[R1,K1,2] = 0
 flow_balance_R1_K1_3: - x[0,R1,K1,1] + x[R1,0,K1,3] + x[R1,C1,K1,3]
   + x[R1,C2,K1,3] + x[R1,R2,K1,3] - v[R1,K1,2] + v[R1,K1,3] = 0
 flow_balance_R1_K1_4: - x[0,R1,K1,2] + x[R1,0,K1,4] + x[R1,C1,K1,4]
   + x[R1,C2,K1,4] + x[R1,R2,K1,4] - x[R2,R1,K1,1] - v[R1,K1,3]
   + v[R1,K1,4] = 0
 flow_balance_R1_K1_5: - x[0,R1,K1,3] + x[R1,0,K1,5] + x[R1,C1,K1,5]
   + x[R1,C2,K1,5] + x[R1,R2,K1,5] - x[R2,R1,K1,2] - v[R1,K1,4]
   + v[R1,K1,5] = 0
 flow_balance_R1_K1_6: - x[0,R1,K1,4] - x[C1,R1,K1,1] - x[C2,R1,K1,1]
   + x[R1,0,K1,6] + x[R1,C1,K1,6] + x[R1,C2,K1,6] + x[R1,R2,K1,6]
   - x[R2,R1,K1,3] - v[R1,K1,5] + v[R1,K1,6] = 0
 flow_balance_R1_K1_7: - x[0,R1,K1,5] - x[C1,R1,K1,2] - x[C2,R1,K1,2]
   + x[R1,0,K1,7] + x[R1,C1,K1,7] + x[R1,C2,K1,7] + x[R1,R2,K1,7]
   - x[R2,R1,K1,4] - v[R1,K1,6] + v[R1,K1,7] = 0
 flow_balance_R1_K1_8: - x[0,R1,K1,6] - x[C1,R1,K1,3] - x[C2,R1,K1,3]
   + x[R1,0,K1,8] + x[R1,C1,K1,8] + x[R1,C2,K1,8] + x[R1,R2,K1,8]
   - x[R2,R1,K1,5] - v[R1,K1,7] + v[R1,K1,8] = 0
 flow_balance_R1_K1_9: - x[0,R1,K1,7] - x[C1,R1,K1,4] - x[C2,R1,K1,4]
   + x[R1,0,K1,9] + x[R1,C1,K1,9] + x[R1,C2,K1,9] + x[R1,R2,K1,9]
   - x[R2,R1,K1,6] - v[R1,K1,8] + v[R1,K1,9] = 0
 flow_balance_R1_K1_10: - x[0,R1,K1,8] - x[C1,R1,K1,5] - x[C2,R1,K1,5]
   + x[R1,0,K1,10] + x[R1,C1,K1,10] + x[R1,C2,K1,10] + x[R1,R2,K1,10]
   - x[R2,R1,K1,7] - v[R1,K1,9] + v[R1,K1,10] = 0
 flow_balance_R1_K1_11: - x[0,R1,K1,9] - x[C1,R1,K1,6] - x[C2,R1,K1,6]
   + x[R1,0,K1,11] + x[R1,C1,K1,11] + x[R1,C2,K1,11] + x[R1,R2,K1,11]
   - x[R2,R1,K1,8] - v[R1,K1,10] + v[R1,K1,11] = 0
 flow_balance_R1_K1_12: - x[0,R1,K1,10] - x[C1,R1,K1,7] - x[C2,R1,K1,7]
   + x[R1,0,K1,12] + x[R1,C1,K1,12] + x[R1,C2,K1,12] + x[R1,R2,K1,12]
   - x[R2,R1,K1,9] - v[R1,K1,11] + v[R1,K1,12] = 0
 flow_balance_R1_K1_13: - x[0,R1,K1,11] - x[C1,R1,K1,8] - x[C2,R1,K1,8]
   + x[R1,0,K1,13] + x[R1,C1,K1,13] + x[R1,C2,K1,13] + x[R1,R2,K1,13]
   - x[R2,R1,K1,10] - v[R1,K1,12] + v[R1,K1,13] = 0
 flow_balance_R1_K1_14: - x[0,R1,K1,12] - x[C1,R1,K1,9] - x[C2,R1,K1,9]
   + x[R1,0,K1,14] + x[R1,C1,K1,14] + x[R1,C2,K1,14] + x[R1,R2,K1,14]
   - x[R2,R1,K1,11] - v[R1,K1,13] + v[R1,K1,14] = 0
 flow_balance_R1_K1_15: - x[0,R1,K1,13] - x[C1,R1,K1,10] - x[C2,R1,K1,10]
   + x[R1,0,K1,15] + x[R1,C1,K1,15] + x[R1,C2,K1,15] + x[R1,R2,K1,15]
   - x[R2,R1,K1,12] - v[R1,K1,14] + v[R1,K1,15] = 0
 flow_balance_R2_K1_2: x[R2,0,K1,2] + x[R2,C1,K1,2] + x[R2,C2,K1,2]
   + x[R2,R1,K1,2] - v[R2,K1,1] + v[R2,K1,2] = 0
 flow_balance_R2_K1_3: x[R2,0,K1,3] + x[R2,C1,K1,3] + x[R2,C2,K1,3]
   + x[R2,R1,K1,3] - v[R2,K1,2] + v[R2,K1,3] = 0
 flow_balance_R2_K1_4: - x[0,R2,K1,1] - x[C1,R2,K1,1] - x[R1,R2,K1,1]
   + x[R2,0,K1,4] + x[R2,C1,K1,4] + x[R2,C2,K1,4] + x[R2,R1,K1,4]
   - v[R2,K1,3] + v[R2,K1,4] = 0
 flow_balance_R2_K1_5: - x[0,R2,K1,2] - x[C1,R2,K1,2] - x[C2,R2,K1,1]
   - x[R1,R2,K1,2] + x[R2,0,K1,5] + x[R2,C1,K1,5] + x[R2,C2,K1,5]
   + x[R2,R1,K1,5] - v[R2,K1,4] + v[R2,K1,5] = 0
 flow_balance_R2_K1_6: - x[0,R2,K1,3] - x[C1,R2,K1,3] - x[C2,R2,K1,2]
   - x[R1,R2,K1,3] + x[R2,0,K1,6] + x[R2,C1,K1,6] + x[R2,C2,K1,6]
   + x[R2,R1,K1,6] - v[R2,K1,5] + v[R2,K1,6] = 0
 flow_balance_R2_K1_7: - x[0,R2,K1,4] - x[C1,R2,K1,4] - x[C2,R2,K1,3]
   - x[R1,R2,K1,4] + x[R2,0,K1,7] + x[R2,C1,K1,7] + x[R2,C2,K1,7]
   + x[R2,R1,K1,7] - v[R2,K1,6] + v[R2,K1,7] = 0
 flow_balance_R2_K1_8: - x[0,R2,K1,5] - x[C1,R2,K1,5] - x[C2,R2,K1,4]
   - x[R1,R2,K1,5] + x[R2,0,K1,8] + x[R2,C1,K1,8] + x[R2,C2,K1,8]
   + x[R2,R1,K1,8] - v[R2,K1,7] + v[R2,K1,8] = 0
 flow_balance_R2_K1_9: - x[0,R2,K1,6] - x[C1,R2,K1,6] - x[C2,R2,K1,5]
   - x[R1,R2,K1,6] + x[R2,0,K1,9] + x[R2,C1,K1,9] + x[R2,C2,K1,9]
   + x[R2,R1,K1,9] - v[R2,K1,8] + v[R2,K1,9] = 0
 flow_balance_R2_K1_10: - x[0,R2,K1,7] - x[C1,R2,K1,7] - x[C2,R2,K1,6]
   - x[R1,R2,K1,7] + x[R2,0,K1,10] + x[R2,C1,K1,10] + x[R2,C2,K1,10]
   + x[R2,R1,K1,10] - v[R2,K1,9] + v[R2,K1,10] = 0
 flow_balance_R2_K1_11: - x[0,R2,K1,8] - x[C1,R2,K1,8] - x[C2,R2,K1,7]
   - x[R1,R2,K1,8] + x[R2,0,K1,11] + x[R2,C1,K1,11] + x[R2,C2,K1,11]
   + x[R2,R1,K1,11] - v[R2,K1,10] + v[R2,K1,11] = 0
 flow_balance_R2_K1_12: - x[0,R2,K1,9] - x[C1,R2,K1,9] - x[C2,R2,K1,8]
   - x[R1,R2,K1,9] + x[R2,0,K1,12] + x[R2,C1,K1,12] + x[R2,C2,K1,12]
   + x[R2,R1,K1,12] - v[R2,K1,11] + v[R2,K1,12] = 0
 flow_balance_R2_K1_13: - x[0,R2,K1,10] - x[C1,R2,K1,10] - x[C2,R2,K1,9]
   - x[R1,R2,K1,10] + x[R2,0,K1,13] + x[R2,C1,K1,13] + x[R2,C2,K1,13]
   + x[R2,R1,K1,13] - v[R2,K1,12] + v[R2,K1,13] = 0
 flow_balance_R2_K1_14: - x[0,R2,K1,11] - x[C1,R2,K1,11] - x[C2,R2,K1,10]
   - x[R1,R2,K1,11] + x[R2,0,K1,14] + x[R2,C1,K1,14] + x[R2,C2,K1,14]
   + x[R2,R1,K1,14] - v[R2,K1,13] + v[R2,K1,14] = 0
 flow_balance_R2_K1_15: - x[0,R2,K1,12] - x[C1,R2,K1,12] - x[C2,R2,K1,11]
   - x[R1,R2,K1,12] + x[R2,0,K1,15] + x[R2,C1,K1,15] + x[R2,C2,K1,15]
   + x[R2,R1,K1,15] - v[R2,K1,14] + v[R2,K1,15] = 0
 flow_balance_0_K2_2: x[0,C1,K2,2] + x[0,C2,K2,2] + x[0,R1,K2,2]
   + x[0,R2,K2,2] - v[0,K2,1] + v[0,K2,2] = 0
 flow_balance_0_K2_3: x[0,C1,K2,3] + x[0,C2,K2,3] + x[0,R1,K2,3]
   + x[0,R2,K2,3] - x[R1,0,K2,1] - v[0,K2,2] + v[0,K2,3] = 0
 flow_balance_0_K2_4: x[0,C1,K2,4] + x[0,C2,K2,4] + x[0,R1,K2,4]
   + x[0,R2,K2,4] - x[C2,0,K2,1] - x[R1,0,K2,2] - x[R2,0,K2,1] - v[0,K2,3]
   + v[0,K2,4] = 0
 flow_balance_0_K2_5: x[0,C1,K2,5] + x[0,C2,K2,5] + x[0,R1,K2,5]
   + x[0,R2,K2,5] - x[C1,0,K2,1] - x[C2,0,K2,2] - x[R1,0,K2,3]
   - x[R2,0,K2,2] - v[0,K2,4] + v[0,K2,5] = 0
 flow_balance_0_K2_6: x[0,C1,K2,6] + x[0,C2,K2,6] + x[0,R1,K2,6]
   + x[0,R2,K2,6] - x[C1,0,K2,2] - x[C2,0,K2,3] - x[R1,0,K2,4]
   - x[R2,0,K2,3] - v[0,K2,5] + v[0,K2,6] = 0
 flow_balance_0_K2_7: x[0,C1,K2,7] + x[0,C2,K2,7] + x[0,R1,K2,7]
   + x[0,R2,K2,7] - x[C1,0,K2,3] - x[C2,0,K2,4] - x[R1,0,K2,5]
   - x[R2,0,K2,4] - v[0,K2,6] + v[0,K2,7] = 0
 flow_balance_0_K2_8: x[0,C1,K2,8] + x[0,C2,K2,8] + x[0,R1,K2,8]
   + x[0,R2,K2,8] - x[C1,0,K2,4] - x[C2,0,K2,5] - x[R1,0,K2,6]
   - x[R2,0,K2,5] - v[0,K2,7] + v[0,K2,8] = 0
 flow_balance_0_K2_9: x[0,C1,K2,9] + x[0,C2,K2,9] + x[0,R1,K2,9]
   + x[0,R2,K2,9] - x[C1,0,K2,5] - x[C2,0,K2,6] - x[R1,0,K2,7]
   - x[R2,0,K2,6] - v[0,K2,8] + v[0,K2,9] = 0
 flow_balance_0_K2_10: x[0,C1,K2,10] + x[0,C2,K2,10] + x[0,R1,K2,10]
   + x[0,R2,K2,10] - x[C1,0,K2,6] - x[C2,0,K2,7] - x[R1,0,K2,8]
   - x[R2,0,K2,7] - v[0,K2,9] + v[0,K2,10] = 0
 flow_balance_0_K2_11: x[0,C1,K2,11] + x[0,C2,K2,11] + x[0,R1,K2,11]
   + x[0,R2,K2,11] - x[C1,0,K2,7] - x[C2,0,K2,8] - x[R1,0,K2,9]
   - x[R2,0,K2,8] - v[0,K2,10] + v[0,K2,11] = 0
 flow_balance_0_K2_12: x[0,C1,K2,12] + x[0,C2,K2,12] + x[0,R1,K2,12]
   + x[0,R2,K2,12] - x[C1,0,K2,8] - x[C2,0,K2,9] - x[R1,0,K2,10]
   - x[R2,0,K2,9] - v[0,K2,11] + v[0,K2,12] = 0
 flow_balance_0_K2_13: x[0,C1,K2,13] + x[0,C2,K2,13] + x[0,R1,K2,13]
   + x[0,R2,K2,13] - x[C1,0,K2,9] - x[C2,0,K2,10] - x[R1,0,K2,11]
   - x[R2,0,K2,10] - v[0,K2,12] + v[0,K2,13] = 0
 flow_balance_0_K2_14: x[0,C1,K2,14] + x[0,C2,K2,14] + x[0,R1,K2,14]
   + x[0,R2,K2,14] - x[C1,0,K2,10] - x[C2,0,K2,11] - x[R1,0,K2,12]
   - x[R2,0,K2,11] - v[0,K2,13] + v[0,K2,14] = 0
 flow_balance_0_K2_15: x[0,C1,K2,15] + x[0,C2,K2,15] + x[0,R1,K2,15]
   + x[0,R2,K2,15] - x[C1,0,K2,11] - x[C2,0,K2,12] - x[R1,0,K2,13]
   - x[R2,0,K2,12] - v[0,K2,14] + v[0,K2,15] = 0
 flow_balance_C1_K2_2: x[C1,0,K2,2] + x[C1,C2,K2,2] + x[C1,R1,K2,2]
   + x[C1,R2,K2,2] - v[C1,K2,1] + v[C1,K2,2] = 0
 flow_balance_C1_K2_3: x[C1,0,K2,3] + x[C1,C2,K2,3] + x[C1,R1,K2,3]
   + x[C1,R2,K2,3] - v[C1,K2,2] + v[C1,K2,3] = 0
 flow_balance_C1_K2_4: x[C1,0,K2,4] + x[C1,C2,K2,4] + x[C1,R1,K2,4]
   + x[C1,R2,K2,4] - x[R2,C1,K2,1] - v[C1,K2,3] + v[C1,K2,4] = 0
 flow_balance_C1_K2_5: - x[0,C1,K2,1] + x[C1,0,K2,5] + x[C1,C2,K2,5]
   + x[C1,R1,K2,5] + x[C1,R2,K2,5] - x[C2,C1,K2,1] - x[R2,C1,K2,2]
   - v[C1,K2,4] + v[C1,K2,5] = 0
 flow_balance_C1_K2_6: - x[0,C1,K2,2] + x[C1,0,K2,6] + x[C1,C2,K2,6]
   + x[C1,R1,K2,6] + x[C1,R2,K2,6] - x[C2,C1,K2,2] - x[R1,C1,K2,1]
   - x[R2,C1,K2,3] - v[C1,K2,5] + v[C1,K2,6] = 0
 flow_balance_C1_K2_7: - x[0,C1,K2,3] + x[C1,0,K2,7] + x[C1,C2,K2,7]
   + x[C1,R1,K2,7] + x[C1,R2,K2,7] - x[C2,C1,K2,3] - x[R1,C1,K2,2]
   - x[R2,C1,K2,4] - v[C1,K2,6] + v[C1,K2,7] = 0
 flow_balance_C1_K2_8: - x[0,C1,K2,4] + x[C1,0,K2,8] + x[C1,C2,K2,8]
   + x[C1,R1,K2,8] + x[C1,R2,K2,8] - x[C2,C1,K2,4] - x[R1,C1,K2,3]
   - x[R2,C1,K2,5] - v[C1,K2,7] + v[C1,K2,8] = 0
 flow_balance_C1_K2_9: - x[0,C1,K2,5] + x[C1,0,K2,9] + x[C1,C2,K2,9]
   + x[C1,R1,K2,9] + x[C1,R2,K2,9] - x[C2,C1,K2,5] - x[R1,C1,K2,4]
   - x[R2,C1,K2,6] - v[C1,K2,8] + v[C1,K2,9] = 0
 flow_balance_C1_K2_10: - x[0,C1,K2,6] + x[C1,0,K2,10] + x[C1,C2,K2,10]
   + x[C1,R1,K2,10] + x[C1,R2,K2,10] - x[C2,C1,K2,6] - x[R1,C1,K2,5]
   - x[R2,C1,K2,7] - v[C1,K2,9] + v[C1,K2,10] = 0
 flow_balance_C1_K2_11: - x[0,C1,K2,7] + x[C1,0,K2,11] + x[C1,C2,K2,11]
   + x[C1,R1,K2,11] + x[C1,R2,K2,11] - x[C2,C1,K2,7] - x[R1,C1,K2,6]
   - x[R2,C1,K2,8] - v[C1,K2,10] + v[C1,K2,11] = 0
 flow_balance_C1_K2_12: - x[0,C1,K2,8] + x[C1,0,K2,12] + x[C1,C2,K2,12]
   + x[C1,R1,K2,12] + x[C1,R2,K2,12] - x[C2,C1,K2,8] - x[R1,C1,K2,7]
   - x[R2,C1,K2,9] - v[C1,K2,11] + v[C1,K2,12] = 0
 flow_balance_C1_K2_13: - x[0,C1,K2,9] + x[C1,0,K2,13] + x[C1,C2,K2,13]
   + x[C1,R1,K2,13] + x[C1,R2,K2,13] - x[C2,C1,K2,9] - x[R1,C1,K2,8]
   - x[R2,C1,K2,10] - v[C1,K2,12] + v[C1,K2,13] = 0
 flow_balance_C1_K2_14: - x[0,C1,K2,10] + x[C1,0,K2,14] + x[C1,C2,K2,14]
   + x[C1,R1,K2,14] + x[C1,R2,K2,14] - x[C2,C1,K2,10] - x[R1,C1,K2,9]
   - x[R2,C1,K2,11] - v[C1,K2,13] + v[C1,K2,14] = 0
 flow_balance_C1_K2_15: - x[0,C1,K2,11] + x[C1,0,K2,15] + x[C1,C2,K2,15]
   + x[C1,R1,K2,15] + x[C1,R2,K2,15] - x[C2,C1,K2,11] - x[R1,C1,K2,10]
   - x[R2,C1,K2,12] - v[C1,K2,14] + v[C1,K2,15] = 0
 flow_balance_C2_K2_2: x[C2,0,K2,2] + x[C2,C1,K2,2] + x[C2,R1,K2,2]
   + x[C2,R2,K2,2] - v[C2,K2,1] + v[C2,K2,2] = 0
 flow_balance_C2_K2_3: x[C2,0,K2,3] + x[C2,C1,K2,3] + x[C2,R1,K2,3]
   + x[C2,R2,K2,3] - v[C2,K2,2] + v[C2,K2,3] = 0
 flow_balance_C2_K2_4: - x[0,C2,K2,1] + x[C2,0,K2,4] + x[C2,C1,K2,4]
   + x[C2,R1,K2,4] + x[C2,R2,K2,4] - v[C2,K2,3] + v[C2,K2,4] = 0
 flow_balance_C2_K2_5: - x[0,C2,K2,2] - x[C1,C2,K2,1] + x[C2,0,K2,5]
   + x[C2,C1,K2,5] + x[C2,R1,K2,5] + x[C2,R2,K2,5] - x[R2,C2,K2,1]
   - v[C2,K2,4] + v[C2,K2,5] = 0
 flow_balance_C2_K2_6: - x[0,C2,K2,3] - x[C1,C2,K2,2] + x[C2,0,K2,6]
   + x[C2,C1,K2,6] + x[C2,R1,K2,6] + x[C2,R2,K2,6] - x[R1,C2,K2,1]
   - x[R2,C2,K2,2] - v[C2,K2,5] + v[C2,K2,6] = 0
 flow_balance_C2_K2_7: - x[0,C2,K2,4] - x[C1,C2,K2,3] + x[C2,0,K2,7]
   + x[C2,C1,K2,7] + x[C2,R1,K2,7] + x[C2,R2,K2,7] - x[R1,C2,K2,2]
   - x[R2,C2,K2,3] - v[C2,K2,6] + v[C2,K2,7] = 0
 flow_balance_C2_K2_8: - x[0,C2,K2,5] - x[C1,C2,K2,4] + x[C2,0,K2,8]
   + x[C2,C1,K2,8] + x[C2,R1,K2,8] + x[C2,R2,K2,8] - x[R1,C2,K2,3]
   - x[R2,C2,K2,4] - v[C2,K2,7] + v[C2,K2,8] = 0
 flow_balance_C2_K2_9: - x[0,C2,K2,6] - x[C1,C2,K2,5] + x[C2,0,K2,9]
   + x[C2,C1,K2,9] + x[C2,R1,K2,9] + x[C2,R2,K2,9] - x[R1,C2,K2,4]
   - x[R2,C2,K2,5] - v[C2,K2,8] + v[C2,K2,9] = 0
 flow_balance_C2_K2_10: - x[0,C2,K2,7] - x[C1,C2,K2,6] + x[C2,0,K2,10]
   + x[C2,C1,K2,10] + x[C2,R1,K2,10] + x[C2,R2,K2,10] - x[R1,C2,K2,5]
   - x[R2,C2,K2,6] - v[C2,K2,9] + v[C2,K2,10] = 0
 flow_balance_C2_K2_11: - x[0,C2,K2,8] - x[C1,C2,K2,7] + x[C2,0,K2,11]
   + x[C2,C1,K2,11] + x[C2,R1,K2,11] + x[C2,R2,K2,11] - x[R1,C2,K2,6]
   - x[R2,C2,K2,7] - v[C2,K2,10] + v[C2,K2,11] = 0
 flow_balance_C2_K2_12: - x[0,C2,K2,9] - x[C1,C2,K2,8] + x[C2,0,K2,12]
   + x[C2,C1,K2,12] + x[C2,R1,K2,12] + x[C2,R2,K2,12] - x[R1,C2,K2,7]
   - x[R2,C2,K2,8] - v[C2,K2,11] + v[C2,K2,12] = 0
 flow_balance_C2_K2_13: - x[0,C2,K2,10] - x[C1,C2,K2,9] + x[C2,0,K2,13]
   + x[C2,C1,K2,13] + x[C2,R1,K2,13] + x[C2,R2,K2,13] - x[R1,C2,K2,8]
   - x[R2,C2,K2,9] - v[C2,K2,12] + v[C2,K2,13] = 0
 flow_balance_C2_K2_14: - x[0,C2,K2,11] - x[C1,C2,K2,10] + x[C2,0,K2,14]
   + x[C2,C1,K2,14] + x[C2,R1,K2,14] + x[C2,R2,K2,14] - x[R1,C2,K2,9]
   - x[R2,C2,K2,10] - v[C2,K2,13] + v[C2,K2,14] = 0
 flow_balance_C2_K2_15: - x[0,C2,K2,12] - x[C1,C2,K2,11] + x[C2,0,K2,15]
   + x[C2,C1,K2,15] + x[C2,R1,K2,15] + x[C2,R2,K2,15] - x[R1,C2,K2,10]
   - x[R2,C2,K2,11] - v[C2,K2,14] + v[C2,K2,15] = 0
 flow_balance_R1_K2_2: x[R1,0,K2,2] + x[R1,C1,K2,2] + x[R1,C2,K2,2]
   + x[R1,R2,K2,2] - v[R1,K2,1] + v[R1,K2,2] = 0
 flow_balance_R1_K2_3: - x[0,R1,K2,1] + x[R1,0,K2,3] + x[R1,C1,K2,3]
   + x[R1,C2,K2,3] + x[R1,R2,K2,3] - v[R1,K2,2] + v[R1,K2,3] = 0
 flow_balance_R1_K2_4: - x[0,R1,K2,2] + x[R1,0,K2,4] + x[R1,C1,K2,4]
   + x[R1,C2,K2,4] + x[R1,R2,K2,4] - x[R2,R1,K2,1] - v[R1,K2,3]
   + v[R1,K2,4] = 0
 flow_balance_R1_K2_5: - x[0,R1,K2,3] + x[R1,0,K2,5] + x[R1,C1,K2,5]
   + x[R1,C2,K2,5] + x[R1,R2,K2,5] - x[R2,R1,K2,2] - v[R1,K2,4]
   + v[R1,K2,5] = 0
 flow_balance_R1_K2_6: - x[0,R1,K2,4] - x[C1,R1,K2,1] - x[C2,R1,K2,1]
   + x[R1,0,K2,6] + x[R1,C1,K2,6] + x[R1,C2,K2,6] + x[R1,R2,K2,6]
   - x[R2,R1,K2,3] - v[R1,K2,5] + v[R1,K2,6] = 0
 flow_balance_R1_K2_7: - x[0,R1,K2,5] - x[C1,R1,K2,2] - x[C2,R1,K2,2]
   + x[R1,0,K2,7] + x[R1,C1,K2,7] + x[R1,C2,K2,7] + x[R1,R2,K2,7]
   - x[R2,R1,K2,4] - v[R1,K2,6] + v[R1,K2,7] = 0
 flow_balance_R1_K2_8: - x[0,R1,K2,6] - x[C1,R1,K2,3] - x[C2,R1,K2,3]
   + x[R1,0,K2,8] + x[R1,C1,K2,8] + x[R1,C2,K2,8] + x[R1,R2,K2,8]
   - x[R2,R1,K2,5] - v[R1,K2,7] + v[R1,K2,8] = 0
 flow_balance_R1_K2_9: - x[0,R1,K2,7] - x[C1,R1,K2,4] - x[C2,R1,K2,4]
   + x[R1,0,K2,9] + x[R1,C1,K2,9] + x[R1,C2,K2,9] + x[R1,R2,K2,9]
   - x[R2,R1,K2,6] - v[R1,K2,8] + v[R1,K2,9] = 0
 flow_balance_R1_K2_10: - x[0,R1,K2,8] - x[C1,R1,K2,5] - x[C2,R1,K2,5]
   + x[R1,0,K2,10] + x[R1,C1,K2,10] + x[R1,C2,K2,10] + x[R1,R2,K2,10]
   - x[R2,R1,K2,7] - v[R1,K2,9] + v[R1,K2,10] = 0
 flow_balance_R1_K2_11: - x[0,R1,K2,9] - x[C1,R1,K2,6] - x[C2,R1,K2,6]
   + x[R1,0,K2,11] + x[R1,C1,K2,11] + x[R1,C2,K2,11] + x[R1,R2,K2,11]
   - x[R2,R1,K2,8] - v[R1,K2,10] + v[R1,K2,11] = 0
 flow_balance_R1_K2_12: - x[0,R1,K2,10] - x[C1,R1,K2,7] - x[C2,R1,K2,7]
   + x[R1,0,K2,12] + x[R1,C1,K2,12] + x[R1,C2,K2,12] + x[R1,R2,K2,12]
   - x[R2,R1,K2,9] - v[R1,K2,11] + v[R1,K2,12] = 0
 flow_balance_R1_K2_13: - x[0,R1,K2,11] - x[C1,R1,K2,8] - x[C2,R1,K2,8]
   + x[R1,0,K2,13] + x[R1,C1,K2,13] + x[R1,C2,K2,13] + x[R1,R2,K2,13]
   - x[R2,R1,K2,10] - v[R1,K2,12] + v[R1,K2,13] = 0
 flow_balance_R1_K2_14: - x[0,R1,K2,12] - x[C1,R1,K2,9] - x[C2,R1,K2,9]
   + x[R1,0,K2,14] + x[R1,C1,K2,14] + x[R1,C2,K2,14] + x[R1,R2,K2,14]
   - x[R2,R1,K2,11] - v[R1,K2,13] + v[R1,K2,14] = 0
 flow_balance_R1_K2_15: - x[0,R1,K2,13] - x[C1,R1,K2,10] - x[C2,R1,K2,10]
   + x[R1,0,K2,15] + x[R1,C1,K2,15] + x[R1,C2,K2,15] + x[R1,R2,K2,15]
   - x[R2,R1,K2,12] - v[R1,K2,14] + v[R1,K2,15] = 0
 flow_balance_R2_K2_2: x[R2,0,K2,2] + x[R2,C1,K2,2] + x[R2,C2,K2,2]
   + x[R2,R1,K2,2] - v[R2,K2,1] + v[R2,K2,2] = 0
 flow_balance_R2_K2_3: x[R2,0,K2,3] + x[R2,C1,K2,3] + x[R2,C2,K2,3]
   + x[R2,R1,K2,3] - v[R2,K2,2] + v[R2,K2,3] = 0
 flow_balance_R2_K2_4: - x[0,R2,K2,1] - x[C1,R2,K2,1] - x[R1,R2,K2,1]
   + x[R2,0,K2,4] + x[R2,C1,K2,4] + x[R2,C2,K2,4] + x[R2,R1,K2,4]
   - v[R2,K2,3] + v[R2,K2,4] = 0
 flow_balance_R2_K2_5: - x[0,R2,K2,2] - x[C1,R2,K2,2] - x[C2,R2,K2,1]
   - x[R1,R2,K2,2] + x[R2,0,K2,5] + x[R2,C1,K2,5] + x[R2,C2,K2,5]
   + x[R2,R1,K2,5] - v[R2,K2,4] + v[R2,K2,5] = 0
 flow_balance_R2_K2_6: - x[0,R2,K2,3] - x[C1,R2,K2,3] - x[C2,R2,K2,2]
   - x[R1,R2,K2,3] + x[R2,0,K2,6] + x[R2,C1,K2,6] + x[R2,C2,K2,6]
   + x[R2,R1,K2,6] - v[R2,K2,5] + v[R2,K2,6] = 0
 flow_balance_R2_K2_7: - x[0,R2,K2,4] - x[C1,R2,K2,4] - x[C2,R2,K2,3]
   - x[R1,R2,K2,4] + x[R2,0,K2,7] + x[R2,C1,K2,7] + x[R2,C2,K2,7]
   + x[R2,R1,K2,7] - v[R2,K2,6] + v[R2,K2,7] = 0
 flow_balance_R2_K2_8: - x[0,R2,K2,5] - x[C1,R2,K2,5] - x[C2,R2,K2,4]
   - x[R1,R2,K2,5] + x[R2,0,K2,8] + x[R2,C1,K2,8] + x[R2,C2,K2,8]
   + x[R2,R1,K2,8] - v[R2,K2,7] + v[R2,K2,8] = 0
 flow_balance_R2_K2_9: - x[0,R2,K2,6] - x[C1,R2,K2,6] - x[C2,R2,K2,5]
   - x[R1,R2,K2,6] + x[R2,0,K2,9] + x[R2,C1,K2,9] + x[R2,C2,K2,9]
   + x[R2,R1,K2,9] - v[R2,K2,8] + v[R2,K2,9] = 0
 flow_balance_R2_K2_10: - x[0,R2,K2,7] - x[C1,R2,K2,7] - x[C2,R2,K2,6]
   - x[R1,R2,K2,7] + x[R2,0,K2,10] + x[R2,C1,K2,10] + x[R2,C2,K2,10]
   + x[R2,R1,K2,10] - v[R2,K2,9] + v[R2,K2,10] = 0
 flow_balance_R2_K2_11: - x[0,R2,K2,8] - x[C1,R2,K2,8] - x[C2,R2,K2,7]
   - x[R1,R2,K2,8] + x[R2,0,K2,11] + x[R2,C1,K2,11] + x[R2,C2,K2,11]
   + x[R2,R1,K2,11] - v[R2,K2,10] + v[R2,K2,11] = 0
 flow_balance_R2_K2_12: - x[0,R2,K2,9] - x[C1,R2,K2,9] - x[C2,R2,K2,8]
   - x[R1,R2,K2,9] + x[R2,0,K2,12] + x[R2,C1,K2,12] + x[R2,C2,K2,12]
   + x[R2,R1,K2,12] - v[R2,K2,11] + v[R2,K2,12] = 0
 flow_balance_R2_K2_13: - x[0,R2,K2,10] - x[C1,R2,K2,10] - x[C2,R2,K2,9]
   - x[R1,R2,K2,10] + x[R2,0,K2,13] + x[R2,C1,K2,13] + x[R2,C2,K2,13]
   + x[R2,R1,K2,13] - v[R2,K2,12] + v[R2,K2,13] = 0
 flow_balance_R2_K2_14: - x[0,R2,K2,11] - x[C1,R2,K2,11] - x[C2,R2,K2,10]
   - x[R1,R2,K2,11] + x[R2,0,K2,14] + x[R2,C1,K2,14] + x[R2,C2,K2,14]
   + x[R2,R1,K2,14] - v[R2,K2,13] + v[R2,K2,14] = 0
 flow_balance_R2_K2_15: - x[0,R2,K2,12] - x[C1,R2,K2,12] - x[C2,R2,K2,11]
   - x[R1,R2,K2,12] + x[R2,0,K2,15] + x[R2,C1,K2,15] + x[R2,C2,K2,15]
   + x[R2,R1,K2,15] - v[R2,K2,14] + v[R2,K2,15] = 0
 visit_K1_C1: v[C1,K1,1] + v[C1,K1,2] + v[C1,K1,3] + v[C1,K1,4]
   + v[C1,K1,5] + v[C1,K1,6] + v[C1,K1,7] + v[C1,K1,8] + v[C1,K1,9]
   + v[C1,K1,10] + v[C1,K1,11] + v[C1,K1,12] + v[C1,K1,13] + v[C1,K1,14]
   + v[C1,K1,15] >= 2
 unload_time_K1: z[K1,1] + z[K1,2] + z[K1,3] + z[K1,4] + z[K1,5] + z[K1,6]
   + z[K1,7] + z[K1,8] + z[K1,9] + z[K1,10] + z[K1,11] + z[K1,12]
   + z[K1,13] + z[K1,14] + z[K1,15] = 2
 z_activation_K1_1: - v[C1,K1,1] + z[K1,1] <= 0
 z_activation_K1_2: - v[C1,K1,2] + z[K1,2] <= 0
 z_activation_K1_3: - v[C1,K1,3] + z[K1,3] <= 0
 z_activation_K1_4: - v[C1,K1,4] + z[K1,4] <= 0
 z_activation_K1_5: - v[C1,K1,5] + z[K1,5] <= 0
 z_activation_K1_6: - v[C1,K1,6] + z[K1,6] <= 0
 z_activation_K1_7: - v[C1,K1,7] + z[K1,7] <= 0
 z_activation_K1_8: - v[C1,K1,8] + z[K1,8] <= 0
 z_activation_K1_9: - v[C1,K1,9] + z[K1,9] <= 0
 z_activation_K1_10: - v[C1,K1,10] + z[K1,10] <= 0
 z_activation_K1_11: - v[C1,K1,11] + z[K1,11] <= 0
 z_activation_K1_12: - v[C1,K1,12] + z[K1,12] <= 0
 z_activation_K1_13: - v[C1,K1,13] + z[K1,13] <= 0
 z_activation_K1_14: - v[C1,K1,14] + z[K1,14] <= 0
 z_activation_K1_15: - v[C1,K1,15] + z[K1,15] <= 0
 visit_K2_C2: v[C2,K2,1] + v[C2,K2,2] + v[C2,K2,3] + v[C2,K2,4]
   + v[C2,K2,5] + v[C2,K2,6] + v[C2,K2,7] + v[C2,K2,8] + v[C2,K2,9]
   + v[C2,K2,10] + v[C2,K2,11] + v[C2,K2,12] + v[C2,K2,13] + v[C2,K2,14]
   + v[C2,K2,15] >= 2
 unload_time_K2: z[K2,1] + z[K2,2] + z[K2,3] + z[K2,4] + z[K2,5] + z[K2,6]
   + z[K2,7] + z[K2,8] + z[K2,9] + z[K2,10] + z[K2,11] + z[K2,12]
   + z[K2,13] + z[K2,14] + z[K2,15] = 2
 z_activation_K2_1: - v[C2,K2,1] + z[K2,1] <= 0
 z_activation_K2_2: - v[C2,K2,2] + z[K2,2] <= 0
 z_activation_K2_3: - v[C2,K2,3] + z[K2,3] <= 0
 z_activation_K2_4: - v[C2,K2,4] + z[K2,4] <= 0
 z_activation_K2_5: - v[C2,K2,5] + z[K2,5] <= 0
 z_activation_K2_6: - v[C2,K2,6] + z[K2,6] <= 0
 z_activation_K2_7: - v[C2,K2,7] + z[K2,7] <= 0
 z_activation_K2_8: - v[C2,K2,8] + z[K2,8] <= 0
 z_activation_K2_9: - v[C2,K2,9] + z[K2,9] <= 0
 z_activation_K2_10: - v[C2,K2,10] + z[K2,10] <= 0
 z_activation_K2_11: - v[C2,K2,11] + z[K2,11] <= 0
 z_activation_K2_12: - v[C2,K2,12] + z[K2,12] <= 0
 z_activation_K2_13: - v[C2,K2,13] + z[K2,13] <= 0
 z_activation_K2_14: - v[C2,K2,14] + z[K2,14] <= 0
 z_activation_K2_15: - v[C2,K2,15] + z[K2,15] <= 0
 bandwidth_K1_1: - 4 z[K1,1] + b[K1,1] = 1
 bandwidth_K1_2: - 4 z[K1,2] + b[K1,2] = 1
 bandwidth_K1_3: - 4 z[K1,3] + b[K1,3] = 1
 bandwidth_K1_4: - 4 z[K1,4] + b[K1,4] = 1
 bandwidth_K1_5: - 4 z[K1,5] + b[K1,5] = 1
 bandwidth_K1_6: - 4 z[K1,6] + b[K1,6] = 1
 bandwidth_K1_7: - 4 z[K1,7] + b[K1,7] = 1
 bandwidth_K1_8: - 4 z[K1,8] + b[K1,8] = 1
 bandwidth_K1_9: - 4 z[K1,9] + b[K1,9] = 1
 bandwidth_K1_10: - 4 z[K1,10] + b[K1,10] = 1
 bandwidth_K1_11: - 4 z[K1,11] + b[K1,11] = 1
 bandwidth_K1_12: - 4 z[K1,12] + b[K1,12] = 1
 bandwidth_K1_13: - 4 z[K1,13] + b[K1,13] = 1
 bandwidth_K1_14: - 4 z[K1,14] + b[K1,14] = 1
 bandwidth_K1_15: - 4 z[K1,15] + b[K1,15] = 1
 bandwidth_K2_1: - 4 z[K2,1] + b[K2,1] = 1
 bandwidth_K2_2: - 4 z[K2,2] + b[K2,2] = 1
 bandwidth_K2_3: - 4 z[K2,3] + b[K2,3] = 1
 bandwidth_K2_4: - 4 z[K2,4] + b[K2,4] = 1
 bandwidth_K2_5: - 4 z[K2,5] + b[K2,5] = 1
 bandwidth_K2_6: - 4 z[K2,6] + b[K2,6] = 1
 bandwidth_K2_7: - 4 z[K2,7] + b[K2,7] = 1
 bandwidth_K2_8: - 4 z[K2,8] + b[K2,8] = 1
 bandwidth_K2_9: - 4 z[K2,9] + b[K2,9] = 1
 bandwidth_K2_10: - 4 z[K2,10] + b[K2,10] = 1
 bandwidth_K2_11: - 4 z[K2,11] + b[K2,11] = 1
 bandwidth_K2_12: - 4 z[K2,12] + b[K2,12] = 1
 bandwidth_K2_13: - 4 z[K2,13] + b[K2,13] = 1
 bandwidth_K2_14: - 4 z[K2,14] + b[K2,14] = 1
 bandwidth_K2_15: - 4 z[K2,15] + b[K2,15] = 1
 comm_source_K1_1: - b[K1,1] + f[0,C1,K1,1] + f[0,C2,K1,1] + f[0,R1,K1,1]
   + f[0,R2,K1,1] - f[C1,0,K1,1] - f[C2,0,K1,1] - f[R1,0,K1,1]
   - f[R2,0,K1,1] = 0
 comm_relay_R1_K1_1: f[0,R1,K1,1] + f[C1,R1,K1,1] + f[C2,R1,K1,1]
   - f[R1,0,K1,1] - f[R1,C1,K1,1] - f[R1,C2,K1,1] - f[R1,R2,K1,1]
   + f[R2,R1,K1,1] = 0
 comm_relay_R2_K1_1: f[0,R2,K1,1] + f[C1,R2,K1,1] + f[C2,R2,K1,1]
   + f[R1,R2,K1,1] - f[R2,0,K1,1] - f[R2,C1,K1,1] - f[R2,C2,K1,1]
   - f[R2,R1,K1,1] = 0
 comm_source_K1_2: - b[K1,2] + f[0,C1,K1,2] + f[0,C2,K1,2] + f[0,R1,K1,2]
   + f[0,R2,K1,2] - f[C1,0,K1,2] - f[C2,0,K1,2] - f[R1,0,K1,2]
   - f[R2,0,K1,2] = 0
 comm_relay_R1_K1_2: f[0,R1,K1,2] + f[C1,R1,K1,2] + f[C2,R1,K1,2]
   - f[R1,0,K1,2] - f[R1,C1,K1,2] - f[R1,C2,K1,2] - f[R1,R2,K1,2]
   + f[R2,R1,K1,2] = 0
 comm_relay_R2_K1_2: f[0,R2,K1,2] + f[C1,R2,K1,2] + f[C2,R2,K1,2]
   + f[R1,R2,K1,2] - f[R2,0,K1,2] - f[R2,C1,K1,2] - f[R2,C2,K1,2]
   - f[R2,R1,K1,2] = 0
 comm_source_K1_3: - b[K1,3] + f[0,C1,K1,3] + f[0,C2,K1,3] + f[0,R1,K1,3]
   + f[0,R2,K1,3] - f[C1,0,K1,3] - f[C2,0,K1,3] - f[R1,0,K1,3]
   - f[R2,0,K1,3] = 0
 comm_relay_R1_K1_3: f[0,R1,K1,3] + f[C1,R1,K1,3] + f[C2,R1,K1,3]
   - f[R1,0,K1,3] - f[R1,C1,K1,3] - f[R1,C2,K1,3] - f[R1,R2,K1,3]
   + f[R2,R1,K1,3] = 0
 comm_relay_R2_K1_3: f[0,R2,K1,3] + f[C1,R2,K1,3] + f[C2,R2,K1,3]
   + f[R1,R2,K1,3] - f[R2,0,K1,3] - f[R2,C1,K1,3] - f[R2,C2,K1,3]
   - f[R2,R1,K1,3] = 0
 comm_source_K1_4: - b[K1,4] + f[0,C1,K1,4] + f[0,C2,K1,4] + f[0,R1,K1,4]
   + f[0,R2,K1,4] - f[C1,0,K1,4] - f[C2,0,K1,4] - f[R1,0,K1,4]
   - f[R2,0,K1,4] = 0
 comm_relay_R1_K1_4: f[0,R1,K1,4] + f[C1,R1,K1,4] + f[C2,R1,K1,4]
   - f[R1,0,K1,4] - f[R1,C1,K1,4] - f[R1,C2,K1,4] - f[R1,R2,K1,4]
   + f[R2,R1,K1,4] = 0
 comm_relay_R2_K1_4: f[0,R2,K1,4] + f[C1,R2,K1,4] + f[C2,R2,K1,4]
   + f[R1,R2,K1,4] - f[R2,0,K1,4] - f[R2,C1,K1,4] - f[R2,C2,K1,4]
   - f[R2,R1,K1,4] = 0
 comm_source_K1_5: - b[K1,5] + f[0,C1,K1,5] + f[0,C2,K1,5] + f[0,R1,K1,5]
   + f[0,R2,K1,5] - f[C1,0,K1,5] - f[C2,0,K1,5] - f[R1,0,K1,5]
   - f[R2,0,K1,5] = 0
 comm_relay_R1_K1_5: f[0,R1,K1,5] + f[C1,R1,K1,5] + f[C2,R1,K1,5]
   - f[R1,0,K1,5] - f[R1,C1,K1,5] - f[R1,C2,K1,5] - f[R1,R2,K1,5]
   + f[R2,R1,K1,5] = 0
 comm_relay_R2_K1_5: f[0,R2,K1,5] + f[C1,R2,K1,5] + f[C2,R2,K1,5]
   + f[R1,R2,K1,5] - f[R2,0,K1,5] - f[R2,C1,K1,5] - f[R2,C2,K1,5]
   - f[R2,R1,K1,5] = 0
 comm_source_K1_6: - b[K1,6] + f[0,C1,K1,6] + f[0,C2,K1,6] + f[0,R1,K1,6]
   + f[0,R2,K1,6] - f[C1,0,K1,6] - f[C2,0,K1,6] - f[R1,0,K1,6]
   - f[R2,0,K1,6] = 0
 comm_relay_R1_K1_6: f[0,R1,K1,6] + f[C1,R1,K1,6] + f[C2,R1,K1,6]
   - f[R1,0,K1,6] - f[R1,C1,K1,6] - f[R1,C2,K1,6] - f[R1,R2,K1,6]
   + f[R2,R1,K1,6] = 0
 comm_relay_R2_K1_6: f[0,R2,K1,6] + f[C1,R2,K1,6] + f[C2,R2,K1,6]
   + f[R1,R2,K1,6] - f[R2,0,K1,6] - f[R2,C1,K1,6] - f[R2,C2,K1,6]
   - f[R2,R1,K1,6] = 0
 comm_source_K1_7: - b[K1,7] + f[0,C1,K1,7] + f[0,C2,K1,7] + f[0,R1,K1,7]
   + f[0,R2,K1,7] - f[C1,0,K1,7] - f[C2,0,K1,7] - f[R1,0,K1,7]
   - f[R2,0,K1,7] = 0
 comm_relay_R1_K1_7: f[0,R1,K1,7] + f[C1,R1,K1,7] + f[C2,R1,K1,7]
   - f[R1,0,K1,7] - f[R1,C1,K1,7] - f[R1,C2,K1,7] - f[R1,R2,K1,7]
   + f[R2,R1,K1,7] = 0
 comm_relay_R2_K1_7: f[0,R2,K1,7] + f[C1,R2,K1,7] + f[C2,R2,K1,7]
   + f[R1,R2,K1,7] - f[R2,0,K1,7] - f[R2,C1,K1,7] - f[R2,C2,K1,7]
   - f[R2,R1,K1,7] = 0
 comm_source_K1_8: - b[K1,8] + f[0,C1,K1,8] + f[0,C2,K1,8] + f[0,R1,K1,8]
   + f[0,R2,K1,8] - f[C1,0,K1,8] - f[C2,0,K1,8] - f[R1,0,K1,8]
   - f[R2,0,K1,8] = 0
 comm_relay_R1_K1_8: f[0,R1,K1,8] + f[C1,R1,K1,8] + f[C2,R1,K1,8]
   - f[R1,0,K1,8] - f[R1,C1,K1,8] - f[R1,C2,K1,8] - f[R1,R2,K1,8]
   + f[R2,R1,K1,8] = 0
 comm_relay_R2_K1_8: f[0,R2,K1,8] + f[C1,R2,K1,8] + f[C2,R2,K1,8]
   + f[R1,R2,K1,8] - f[R2,0,K1,8] - f[R2,C1,K1,8] - f[R2,C2,K1,8]
   - f[R2,R1,K1,8] = 0
 comm_source_K1_9: - b[K1,9] + f[0,C1,K1,9] + f[0,C2,K1,9] + f[0,R1,K1,9]
   + f[0,R2,K1,9] - f[C1,0,K1,9] - f[C2,0,K1,9] - f[R1,0,K1,9]
   - f[R2,0,K1,9] = 0
 comm_relay_R1_K1_9: f[0,R1,K1,9] + f[C1,R1,K1,9] + f[C2,R1,K1,9]
   - f[R1,0,K1,9] - f[R1,C1,K1,9] - f[R1,C2,K1,9] - f[R1,R2,K1,9]
   + f[R2,R1,K1,9] = 0
 comm_relay_R2_K1_9: f[0,R2,K1,9] + f[C1,R2,K1,9] + f[C2,R2,K1,9]
   + f[R1,R2,K1,9] - f[R2,0,K1,9] - f[R2,C1,K1,9] - f[R2,C2,K1,9]
   - f[R2,R1,K1,9] = 0
 comm_source_K1_10: - b[K1,10] + f[0,C1,K1,10] + f[0,C2,K1,10]
   + f[0,R1,K1,10] + f[0,R2,K1,10] - f[C1,0,K1,10] - f[C2,0,K1,10]
   - f[R1,0,K1,10] - f[R2,0,K1,10] = 0
 comm_relay_R1_K1_10: f[0,R1,K1,10] + f[C1,R1,K1,10] + f[C2,R1,K1,10]
   - f[R1,0,K1,10] - f[R1,C1,K1,10] - f[R1,C2,K1,10] - f[R1,R2,K1,10]
   + f[R2,R1,K1,10] = 0
 comm_relay_R2_K1_10: f[0,R2,K1,10] + f[C1,R2,K1,10] + f[C2,R2,K1,10]
   + f[R1,R2,K1,10] - f[R2,0,K1,10] - f[R2,C1,K1,10] - f[R2,C2,K1,10]
   - f[R2,R1,K1,10] = 0
 comm_source_K1_11: - b[K1,11] + f[0,C1,K1,11] + f[0,C2,K1,11]
   + f[0,R1,K1,11] + f[0,R2,K1,11] - f[C1,0,K1,11] - f[C2,0,K1,11]
   - f[R1,0,K1,11] - f[R2,0,K1,11] = 0
 comm_relay_R1_K1_11: f[0,R1,K1,11] + f[C1,R1,K1,11] + f[C2,R1,K1,11]
   - f[R1,0,K1,11] - f[R1,C1,K1,11] - f[R1,C2,K1,11] - f[R1,R2,K1,11]
   + f[R2,R1,K1,11] = 0
 comm_relay_R2_K1_11: f[0,R2,K1,11] + f[C1,R2,K1,11] + f[C2,R2,K1,11]
   + f[R1,R2,K1,11] - f[R2,0,K1,11] - f[R2,C1,K1,11] - f[R2,C2,K1,11]
   - f[R2,R1,K1,11] = 0
 comm_source_K1_12: - b[K1,12] + f[0,C1,K1,12] + f[0,C2,K1,12]
   + f[0,R1,K1,12] + f[0,R2,K1,12] - f[C1,0,K1,12] - f[C2,0,K1,12]
   - f[R1,0,K1,12] - f[R2,0,K1,12] = 0
 comm_relay_R1_K1_12: f[0,R1,K1,12] + f[C1,R1,K1,12] + f[C2,R1,K1,12]
   - f[R1,0,K1,12] - f[R1,C1,K1,12] - f[R1,C2,K1,12] - f[R1,R2,K1,12]
   + f[R2,R1,K1,12] = 0
 comm_relay_R2_K1_12: f[0,R2,K1,12] + f[C1,R2,K1,12] + f[C2,R2,K1,12]
   + f[R1,R2,K1,12] - f[R2,0,K1,12] - f[R2,C1,K1,12] - f[R2,C2,K1,12]
   - f[R2,R1,K1,12] = 0
 comm_source_K1_13: - b[K1,13] + f[0,C1,K1,13] + f[0,C2,K1,13]
   + f[0,R1,K1,13] + f[0,R2,K1,13] - f[C1,0,K1,13] - f[C2,0,K1,13]
   - f[R1,0,K1,13] - f[R2,0,K1,13] = 0
 comm_relay_R1_K1_13: f[0,R1,K1,13] + f[C1,R1,K1,13] + f[C2,R1,K1,13]
   - f[R1,0,K1,13] - f[R1,C1,K1,13] - f[R1,C2,K1,13] - f[R1,R2,K1,13]
   + f[R2,R1,K1,13] = 0
 comm_relay_R2_K1_13: f[0,R2,K1,13] + f[C1,R2,K1,13] + f[C2,R2,K1,13]
   + f[R1,R2,K1,13] - f[R2,0,K1,13] - f[R2,C1,K1,13] - f[R2,C2,K1,13]
   - f[R2,R1,K1,13] = 0
 comm_source_K1_14: - b[K1,14] + f[0,C1,K1,14] + f[0,C2,K1,14]
   + f[0,R1,K1,14] + f[0,R2,K1,14] - f[C1,0,K1,14] - f[C2,0,K1,14]
   - f[R1,0,K1,14] - f[R2,0,K1,14] = 0
 comm_relay_R1_K1_14: f[0,R1,K1,14] + f[C1,R1,K1,14] + f[C2,R1,K1,14]
   - f[R1,0,K1,14] - f[R1,C1,K1,14] - f[R1,C2,K1,14] - f[R1,R2,K1,14]
   + f[R2,R1,K1,14] = 0
 comm_relay_R2_K1_14: f[0,R2,K1,14] + f[C1,R2,K1,14] + f[C2,R2,K1,14]
   + f[R1,R2,K1,14] - f[R2,0,K1,14] - f[R2,C1,K1,14] - f[R2,C2,K1,14]
   - f[R2,R1,K1,14] = 0
 comm_source_K1_15: - b[K1,15] + f[0,C1,K1,15] + f[0,C2,K1,15]
   + f[0,R1,K1,15] + f[0,R2,K1,15] - f[C1,0,K1,15] - f[C2,0,K1,15]
   - f[R1,0,K1,15] - f[R2,0,K1,15] = 0
 comm_relay_R1_K1_15: f[0,R1,K1,15] + f[C1,R1,K1,15] + f[C2,R1,K1,15]
   - f[R1,0,K1,15] - f[R1,C1,K1,15] - f[R1,C2,K1,15] - f[R1,R2,K1,15]
   + f[R2,R1,K1,15] = 0
 comm_relay_R2_K1_15: f[0,R2,K1,15] + f[C1,R2,K1,15] + f[C2,R2,K1,15]
   + f[R1,R2,K1,15] - f[R2,0,K1,15] - f[R2,C1,K1,15] - f[R2,C2,K1,15]
   - f[R2,R1,K1,15] = 0
 comm_source_K2_1: - b[K2,1] + f[0,C1,K2,1] + f[0,C2,K2,1] + f[0,R1,K2,1]
   + f[0,R2,K2,1] - f[C1,0,K2,1] - f[C2,0,K2,1] - f[R1,0,K2,1]
   - f[R2,0,K2,1] = 0
 comm_relay_R1_K2_1: f[0,R1,K2,1] + f[C1,R1,K2,1] + f[C2,R1,K2,1]
   - f[R1,0,K2,1] - f[R1,C1,K2,1] - f[R1,C2,K2,1] - f[R1,R2,K2,1]
   + f[R2,R1,K2,1] = 0
 comm_relay_R2_K2_1: f[0,R2,K2,1] + f[C1,R2,K2,1] + f[C2,R2,K2,1]
   + f[R1,R2,K2,1] - f[R2,0,K2,1] - f[R2,C1,K2,1] - f[R2,C2,K2,1]
   - f[R2,R1,K2,1] = 0
 comm_source_K2_2: - b[K2,2] + f[0,C1,K2,2] + f[0,C2,K2,2] + f[0,R1,K2,2]
   + f[0,R2,K2,2] - f[C1,0,K2,2] - f[C2,0,K2,2] - f[R1,0,K2,2]
   - f[R2,0,K2,2] = 0
 comm_relay_R1_K2_2: f[0,R1,K2,2] + f[C1,R1,K2,2] + f[C2,R1,K2,2]
   - f[R1,0,K2,2] - f[R1,C1,K2,2] - f[R1,C2,K2,2] - f[R1,R2,K2,2]
   + f[R2,R1,K2,2] = 0
 comm_relay_R2_K2_2: f[0,R2,K2,2] + f[C1,R2,K2,2] + f[C2,R2,K2,2]
   + f[R1,R2,K2,2] - f[R2,0,K2,2] - f[R2,C1,K2,2] - f[R2,C2,K2,2]
   - f[R2,R1,K2,2] = 0
 comm_source_K2_3: - b[K2,3] + f[0,C1,K2,3] + f[0,C2,K2,3] + f[0,R1,K2,3]
   + f[0,R2,K2,3] - f[C1,0,K2,3] - f[C2,0,K2,3] - f[R1,0,K2,3]
   - f[R2,0,K2,3] = 0
 comm_relay_R1_K2_3: f[0,R1,K2,3] + f[C1,R1,K2,3] + f[C2,R1,K2,3]
   - f[R1,0,K2,3] - f[R1,C1,K2,3] - f[R1,C2,K2,3] - f[R1,R2,K2,3]
   + f[R2,R1,K2,3] = 0
 comm_relay_R2_K2_3: f[0,R2,K2,3] + f[C1,R2,K2,3] + f[C2,R2,K2,3]
   + f[R1,R2,K2,3] - f[R2,0,K2,3] - f[R2,C1,K2,3] - f[R2,C2,K2,3]
   - f[R2,R1,K2,3] = 0
 comm_source_K2_4: - b[K2,4] + f[0,C1,K2,4] + f[0,C2,K2,4] + f[0,R1,K2,4]
   + f[0,R2,K2,4] - f[C1,0,K2,4] - f[C2,0,K2,4] - f[R1,0,K2,4]
   - f[R2,0,K2,4] = 0
 comm_relay_R1_K2_4: f[0,R1,K2,4] + f[C1,R1,K2,4] + f[C2,R1,K2,4]
   - f[R1,0,K2,4] - f[R1,C1,K2,4] - f[R1,C2,K2,4] - f[R1,R2,K2,4]
   + f[R2,R1,K2,4] = 0
 comm_relay_R2_K2_4: f[0,R2,K2,4] + f[C1,R2,K2,4] + f[C2,R2,K2,4]
   + f[R1,R2,K2,4] - f[R2,0,K2,4] - f[R2,C1,K2,4] - f[R2,C2,K2,4]
   - f[R2,R1,K2,4] = 0
 comm_source_K2_5: - b[K2,5] + f[0,C1,K2,5] + f[0,C2,K2,5] + f[0,R1,K2,5]
   + f[0,R2,K2,5] - f[C1,0,K2,5] - f[C2,0,K2,5] - f[R1,0,K2,5]
   - f[R2,0,K2,5] = 0
 comm_relay_R1_K2_5: f[0,R1,K2,5] + f[C1,R1,K2,5] + f[C2,R1,K2,5]
   - f[R1,0,K2,5] - f[R1,C1,K2,5] - f[R1,C2,K2,5] - f[R1,R2,K2,5]
   + f[R2,R1,K2,5] = 0
 comm_relay_R2_K2_5: f[0,R2,K2,5] + f[C1,R2,K2,5] + f[C2,R2,K2,5]
   + f[R1,R2,K2,5] - f[R2,0,K2,5] - f[R2,C1,K2,5] - f[R2,C2,K2,5]
   - f[R2,R1,K2,5] = 0
 comm_source_K2_6: - b[K2,6] + f[0,C1,K2,6] + f[0,C2,K2,6] + f[0,R1,K2,6]
   + f[0,R2,K2,6] - f[C1,0,K2,6] - f[C2,0,K2,6] - f[R1,0,K2,6]
   - f[R2,0,K2,6] = 0
 comm_relay_R1_K2_6: f[0,R1,K2,6] + f[C1,R1,K2,6] + f[C2,R1,K2,6]
   - f[R1,0,K2,6] - f[R1,C1,K2,6] - f[R1,C2,K2,6] - f[R1,R2,K2,6]
   + f[R2,R1,K2,6] = 0
 comm_relay_R2_K2_6: f[0,R2,K2,6] + f[C1,R2,K2,6] + f[C2,R2,K2,6]
   + f[R1,R2,K2,6] - f[R2,0,K2,6] - f[R2,C1,K2,6] - f[R2,C2,K2,6]
   - f[R2,R1,K2,6] = 0
 comm_source_K2_7: - b[K2,7] + f[0,C1,K2,7] + f[0,C2,K2,7] + f[0,R1,K2,7]
   + f[0,R2,K2,7] - f[C1,0,K2,7] - f[C2,0,K2,7] - f[R1,0,K2,7]
   - f[R2,0,K2,7] = 0
 comm_relay_R1_K2_7: f[0,R1,K2,7] + f[C1,R1,K2,7] + f[C2,R1,K2,7]
   - f[R1,0,K2,7] - f[R1,C1,K2,7] - f[R1,C2,K2,7] - f[R1,R2,K2,7]
   + f[R2,R1,K2,7] = 0
 comm_relay_R2_K2_7: f[0,R2,K2,7] + f[C1,R2,K2,7] + f[C2,R2,K2,7]
   + f[R1,R2,K2,7] - f[R2,0,K2,7] - f[R2,C1,K2,7] - f[R2,C2,K2,7]
   - f[R2,R1,K2,7] = 0
 comm_source_K2_8: - b[K2,8] + f[0,C1,K2,8] + f[0,C2,K2,8] + f[0,R1,K2,8]
   + f[0,R2,K2,8] - f[C1,0,K2,8] - f[C2,0,K2,8] - f[R1,0,K2,8]
   - f[R2,0,K2,8] = 0
 comm_relay_R1_K2_8: f[0,R1,K2,8] + f[C1,R1,K2,8] + f[C2,R1,K2,8]
   - f[R1,0,K2,8] - f[R1,C1,K2,8] - f[R1,C2,K2,8] - f[R1,R2,K2,8]
   + f[R2,R1,K2,8] = 0
 comm_relay_R2_K2_8: f[0,R2,K2,8] + f[C1,R2,K2,8] + f[C2,R2,K2,8]
   + f[R1,R2,K2,8] - f[R2,0,K2,8] - f[R2,C1,K2,8] - f[R2,C2,K2,8]
   - f[R2,R1,K2,8] = 0
 comm_source_K2_9: - b[K2,9] + f[0,C1,K2,9] + f[0,C2,K2,9] + f[0,R1,K2,9]
   + f[0,R2,K2,9] - f[C1,0,K2,9] - f[C2,0,K2,9] - f[R1,0,K2,9]
   - f[R2,0,K2,9] = 0
 comm_relay_R1_K2_9: f[0,R1,K2,9] + f[C1,R1,K2,9] + f[C2,R1,K2,9]
   - f[R1,0,K2,9] - f[R1,C1,K2,9] - f[R1,C2,K2,9] - f[R1,R2,K2,9]
   + f[R2,R1,K2,9] = 0
 comm_relay_R2_K2_9: f[0,R2,K2,9] + f[C1,R2,K2,9] + f[C2,R2,K2,9]
   + f[R1,R2,K2,9] - f[R2,0,K2,9] - f[R2,C1,K2,9] - f[R2,C2,K2,9]
   - f[R2,R1,K2,9] = 0
 comm_source_K2_10: - b[K2,10] + f[0,C1,K2,10] + f[0,C2,K2,10]
   + f[0,R1,K2,10] + f[0,R2,K2,10] - f[C1,0,K2,10] - f[C2,0,K2,10]
   - f[R1,0,K2,10] - f[R2,0,K2,10] = 0
 comm_relay_R1_K2_10: f[0,R1,K2,10] + f[C1,R1,K2,10] + f[C2,R1,K2,10]
   - f[R1,0,K2,10] - f[R1,C1,K2,10] - f[R1,C2,K2,10] - f[R1,R2,K2,10]
   + f[R2,R1,K2,10] = 0
 comm_relay_R2_K2_10: f[0,R2,K2,10] + f[C1,R2,K2,10] + f[C2,R2,K2,10]
   + f[R1,R2,K2,10] - f[R2,0,K2,10] - f[R2,C1,K2,10] - f[R2,C2,K2,10]
   - f[R2,R1,K2,10] = 0
 comm_source_K2_11: - b[K2,11] + f[0,C1,K2,11] + f[0,C2,K2,11]
   + f[0,R1,K2,11] + f[0,R2,K2,11] - f[C1,0,K2,11] - f[C2,0,K2,11]
   - f[R1,0,K2,11] - f[R2,0,K2,11] = 0
 comm_relay_R1_K2_11: f[0,R1,K2,11] + f[C1,R1,K2,11] + f[C2,R1,K2,11]
   - f[R1,0,K2,11] - f[R1,C1,K2,11] - f[R1,C2,K2,11] - f[R1,R2,K2,11]
   + f[R2,R1,K2,11] = 0
 comm_relay_R2_K2_11: f[0,R2,K2,11] + f[C1,R2,K2,11] + f[C2,R2,K2,11]
   + f[R1,R2,K2,11] - f[R2,0,K2,11] - f[R2,C1,K2,11] - f[R2,C2,K2,11]
   - f[R2,R1,K2,11] = 0
 comm_source_K2_12: - b[K2,12] + f[0,C1,K2,12] + f[0,C2,K2,12]
   + f[0,R1,K2,12] + f[0,R2,K2,12] - f[C1,0,K2,12] - f[C2,0,K2,12]
   - f[R1,0,K2,12] - f[R2,0,K2,12] = 0
 comm_relay_R1_K2_12: f[0,R1,K2,12] + f[C1,R1,K2,12] + f[C2,R1,K2,12]
   - f[R1,0,K2,12] - f[R1,C1,K2,12] - f[R1,C2,K2,12] - f[R1,R2,K2,12]
   + f[R2,R1,K2,12] = 0
 comm_relay_R2_K2_12: f[0,R2,K2,12] + f[C1,R2,K2,12] + f[C2,R2,K2,12]
   + f[R1,R2,K2,12] - f[R2,0,K2,12] - f[R2,C1,K2,12] - f[R2,C2,K2,12]
   - f[R2,R1,K2,12] = 0
 comm_source_K2_13: - b[K2,13] + f[0,C1,K2,13] + f[0,C2,K2,13]
   + f[0,R1,K2,13] + f[0,R2,K2,13] - f[C1,0,K2,13] - f[C2,0,K2,13]
   - f[R1,0,K2,13] - f[R2,0,K2,13] = 0
 comm_relay_R1_K2_13: f[0,R1,K2,13] + f[C1,R1,K2,13] + f[C2,R1,K2,13]
   - f[R1,0,K2,13] - f[R1,C1,K2,13] - f[R1,C2,K2,13] - f[R1,R2,K2,13]
   + f[R2,R1,K2,13] = 0
 comm_relay_R2_K2_13: f[0,R2,K2,13] + f[C1,R2,K2,13] + f[C2,R2,K2,13]
   + f[R1,R2,K2,13] - f[R2,0,K2,13] - f[R2,C1,K2,13] - f[R2,C2,K2,13]
   - f[R2,R1,K2,13] = 0
 comm_source_K2_14: - b[K2,14] + f[0,C1,K2,14] + f[0,C2,K2,14]
   + f[0,R1,K2,14] + f[0,R2,K2,14] - f[C1,0,K2,14] - f[C2,0,K2,14]
   - f[R1,0,K2,14] - f[R2,0,K2,14] = 0
 comm_relay_R1_K2_14: f[0,R1,K2,14] + f[C1,R1,K2,14] + f[C2,R1,K2,14]
   - f[R1,0,K2,14] - f[R1,C1,K2,14] - f[R1,C2,K2,14] - f[R1,R2,K2,14]
   + f[R2,R1,K2,14] = 0
 comm_relay_R2_K2_14: f[0,R2,K2,14] + f[C1,R2,K2,14] + f[C2,R2,K2,14]
   + f[R1,R2,K2,14] - f[R2,0,K2,14] - f[R2,C1,K2,14] - f[R2,C2,K2,14]
   - f[R2,R1,K2,14] = 0
 comm_source_K2_15: - b[K2,15] + f[0,C1,K2,15] + f[0,C2,K2,15]
   + f[0,R1,K2,15] + f[0,R2,K2,15] - f[C1,0,K2,15] - f[C2,0,K2,15]
   - f[R1,0,K2,15] - f[R2,0,K2,15] = 0
 comm_relay_R1_K2_15: f[0,R1,K2,15] + f[C1,R1,K2,15] + f[C2,R1,K2,15]
   - f[R1,0,K2,15] - f[R1,C1,K2,15] - f[R1,C2,K2,15] - f[R1,R2,K2,15]
   + f[R2,R1,K2,15] = 0
 comm_relay_R2_K2_15: f[0,R2,K2,15] + f[C1,R2,K2,15] + f[C2,R2,K2,15]
   + f[R1,R2,K2,15] - f[R2,0,K2,15] - f[R2,C1,K2,15] - f[R2,C2,K2,15]
   - f[R2,R1,K2,15] = 0
 cap_limit_N_0_C1_1: f[0,C1,K1,1] + f[0,C1,K2,1] + f[C1,0,K1,1]
   + f[C1,0,K2,1] <= 2.55
 cap_limit_N_0_C2_1: f[0,C2,K1,1] + f[0,C2,K2,1] + f[C2,0,K1,1]
   + f[C2,0,K2,1] <= 4.913
 cap_limit_R_in_0_R1_1: - 28.689 y[R1,1] + f[0,R1,K1,1] + f[0,R1,K2,1]
   + f[R1,0,K1,1] + f[R1,0,K2,1] <= 0
 cap_limit_R_in_0_R2_1: - 7.706 y[R2,1] + f[0,R2,K1,1] + f[0,R2,K2,1]
   + f[R2,0,K1,1] + f[R2,0,K2,1] <= 0
 cap_limit_N_C1_0_1: f[0,C1,K1,1] + f[0,C1,K2,1] + f[C1,0,K1,1]
   + f[C1,0,K2,1] <= 2.55
 cap_limit_N_C1_C2_1: f[C1,C2,K1,1] + f[C1,C2,K2,1] + f[C2,C1,K1,1]
   + f[C2,C1,K2,1] <= 3.632
 cap_limit_R_in_C1_R1_1: - 1.967 y[R1,1] + f[C1,R1,K1,1] + f[C1,R1,K2,1]
   + f[R1,C1,K1,1] + f[R1,C1,K2,1] <= 0
 cap_limit_R_in_C1_R2_1: - 8.302 y[R2,1] + f[C1,R2,K1,1] + f[C1,R2,K2,1]
   + f[R2,C1,K1,1] + f[R2,C1,K2,1] <= 0
 cap_limit_N_C2_0_1: f[0,C2,K1,1] + f[0,C2,K2,1] + f[C2,0,K1,1]
   + f[C2,0,K2,1] <= 4.913
 cap_limit_N_C2_C1_1: f[C1,C2,K1,1] + f[C1,C2,K2,1] + f[C2,C1,K1,1]
   + f[C2,C1,K2,1] <= 3.632
 cap_limit_R_in_C2_R1_1: - 2.489 y[R1,1] + f[C2,R1,K1,1] + f[C2,R1,K2,1]
   + f[R1,C2,K1,1] + f[R1,C2,K2,1] <= 0
 cap_limit_R_in_C2_R2_1: - 3.43 y[R2,1] + f[C2,R2,K1,1] + f[C2,R2,K2,1]
   + f[R2,C2,K1,1] + f[R2,C2,K2,1] <= 0
 cap_limit_R_out_R1_0_1: - 28.689 y[R1,1] + f[0,R1,K1,1] + f[0,R1,K2,1]
   + f[R1,0,K1,1] + f[R1,0,K2,1] <= 0
 cap_limit_R_out_R1_C1_1: - 1.967 y[R1,1] + f[C1,R1,K1,1] + f[C1,R1,K2,1]
   + f[R1,C1,K1,1] + f[R1,C1,K2,1] <= 0
 cap_limit_R_out_R1_C2_1: - 2.489 y[R1,1] + f[C2,R1,K1,1] + f[C2,R1,K2,1]
   + f[R1,C2,K1,1] + f[R1,C2,K2,1] <= 0
 cap_limit_R_out_R1_R2_1: - 6.719 y[R1,1] + f[R1,R2,K1,1] + f[R1,R2,K2,1]
   + f[R2,R1,K1,1] + f[R2,R1,K2,1] <= 0
 cap_limit_R_out_R2_0_1: - 7.706 y[R2,1] + f[0,R2,K1,1] + f[0,R2,K2,1]
   + f[R2,0,K1,1] + f[R2,0,K2,1] <= 0
 cap_limit_R_out_R2_C1_1: - 8.302 y[R2,1] + f[C1,R2,K1,1] + f[C1,R2,K2,1]
   + f[R2,C1,K1,1] + f[R2,C1,K2,1] <= 0
 cap_limit_R_out_R2_C2_1: - 3.43 y[R2,1] + f[C2,R2,K1,1] + f[C2,R2,K2,1]
   + f[R2,C2,K1,1] + f[R2,C2,K2,1] <= 0
 cap_limit_R_out_R2_R1_1: - 6.719 y[R2,1] + f[R1,R2,K1,1] + f[R1,R2,K2,1]
   + f[R2,R1,K1,1] + f[R2,R1,K2,1] <= 0
 cap_limit_N_0_C1_2: f[0,C1,K1,2] + f[0,C1,K2,2] + f[C1,0,K1,2]
   + f[C1,0,K2,2] <= 2.55
 cap_limit_N_0_C2_2: f[0,C2,K1,2] + f[0,C2,K2,2] + f[C2,0,K1,2]
   + f[C2,0,K2,2] <= 4.913
 cap_limit_R_in_0_R1_2: - 28.689 y[R1,2] + f[0,R1,K1,2] + f[0,R1,K2,2]
   + f[R1,0,K1,2] + f[R1,0,K2,2] <= 0
 cap_limit_R_in_0_R2_2: - 7.706 y[R2,2] + f[0,R2,K1,2] + f[0,R2,K2,2]
   + f[R2,0,K1,2] + f[R2,0,K2,2] <= 0
 cap_limit_N_C1_0_2: f[0,C1,K1,2] + f[0,C1,K2,2] + f[C1,0,K1,2]
   + f[C1,0,K2,2] <= 2.55
 cap_limit_N_C1_C2_2: f[C1,C2,K1,2] + f[C1,C2,K2,2] + f[C2,C1,K1,2]
   + f[C2,C1,K2,2] <= 3.632
 cap_limit_R_in_C1_R1_2: - 1.967 y[R1,2] + f[C1,R1,K1,2] + f[C1,R1,K2,2]
   + f[R1,C1,K1,2] + f[R1,C1,K2,2] <= 0
 cap_limit_R_in_C1_R2_2: - 8.302 y[R2,2] + f[C1,R2,K1,2] + f[C1,R2,K2,2]
   + f[R2,C1,K1,2] + f[R2,C1,K2,2] <= 0
 cap_limit_N_C2_0_2: f[0,C2,K1,2] + f[0,C2,K2,2] + f[C2,0,K1,2]
   + f[C2,0,K2,2] <= 4.913
 cap_limit_N_C2_C1_2: f[C1,C2,K1,2] + f[C1,C2,K2,2] + f[C2,C1,K1,2]
   + f[C2,C1,K2,2] <= 3.632
 cap_limit_R_in_C2_R1_2: - 2.489 y[R1,2] + f[C2,R1,K1,2] + f[C2,R1,K2,2]
   + f[R1,C2,K1,2] + f[R1,C2,K2,2] <= 0
 cap_limit_R_in_C2_R2_2: - 3.43 y[R2,2] + f[C2,R2,K1,2] + f[C2,R2,K2,2]
   + f[R2,C2,K1,2] + f[R2,C2,K2,2] <= 0
 cap_limit_R_out_R1_0_2: - 28.689 y[R1,2] + f[0,R1,K1,2] + f[0,R1,K2,2]
   + f[R1,0,K1,2] + f[R1,0,K2,2] <= 0
 cap_limit_R_out_R1_C1_2: - 1.967 y[R1,2] + f[C1,R1,K1,2] + f[C1,R1,K2,2]
   + f[R1,C1,K1,2] + f[R1,C1,K2,2] <= 0
 cap_limit_R_out_R1_C2_2: - 2.489 y[R1,2] + f[C2,R1,K1,2] + f[C2,R1,K2,2]
   + f[R1,C2,K1,2] + f[R1,C2,K2,2] <= 0
 cap_limit_R_out_R1_R2_2: - 6.719 y[R1,2] + f[R1,R2,K1,2] + f[R1,R2,K2,2]
   + f[R2,R1,K1,2] + f[R2,R1,K2,2] <= 0
 cap_limit_R_out_R2_0_2: - 7.706 y[R2,2] + f[0,R2,K1,2] + f[0,R2,K2,2]
   + f[R2,0,K1,2] + f[R2,0,K2,2] <= 0
 cap_limit_R_out_R2_C1_2: - 8.302 y[R2,2] + f[C1,R2,K1,2] + f[C1,R2,K2,2]
   + f[R2,C1,K1,2] + f[R2,C1,K2,2] <= 0
 cap_limit_R_out_R2_C2_2: - 3.43 y[R2,2] + f[C2,R2,K1,2] + f[C2,R2,K2,2]
   + f[R2,C2,K1,2] + f[R2,C2,K2,2] <= 0
 cap_limit_R_out_R2_R1_2: - 6.719 y[R2,2] + f[R1,R2,K1,2] + f[R1,R2,K2,2]
   + f[R2,R1,K1,2] + f[R2,R1,K2,2] <= 0
 cap_limit_N_0_C1_3: f[0,C1,K1,3] + f[0,C1,K2,3] + f[C1,0,K1,3]
   + f[C1,0,K2,3] <= 2.55
 cap_limit_N_0_C2_3: f[0,C2,K1,3] + f[0,C2,K2,3] + f[C2,0,K1,3]
   + f[C2,0,K2,3] <= 4.913
 cap_limit_R_in_0_R1_3: - 28.689 y[R1,3] + f[0,R1,K1,3] + f[0,R1,K2,3]
   + f[R1,0,K1,3] + f[R1,0,K2,3] <= 0
 cap_limit_R_in_0_R2_3: - 7.706 y[R2,3] + f[0,R2,K1,3] + f[0,R2,K2,3]
   + f[R2,0,K1,3] + f[R2,0,K2,3] <= 0
 cap_limit_N_C1_0_3: f[0,C1,K1,3] + f[0,C1,K2,3] + f[C1,0,K1,3]
   + f[C1,0,K2,3] <= 2.55
 cap_limit_N_C1_C2_3: f[C1,C2,K1,3] + f[C1,C2,K2,3] + f[C2,C1,K1,3]
   + f[C2,C1,K2,3] <= 3.632
 cap_limit_R_in_C1_R1_3: - 1.967 y[R1,3] + f[C1,R1,K1,3] + f[C1,R1,K2,3]
   + f[R1,C1,K1,3] + f[R1,C1,K2,3] <= 0
 cap_limit_R_in_C1_R2_3: - 8.302 y[R2,3] + f[C1,R2,K1,3] + f[C1,R2,K2,3]
   + f[R2,C1,K1,3] + f[R2,C1,K2,3] <= 0
 cap_limit_N_C2_0_3: f[0,C2,K1,3] + f[0,C2,K2,3] + f[C2,0,K1,3]
   + f[C2,0,K2,3] <= 4.913
 cap_limit_N_C2_C1_3: f[C1,C2,K1,3] + f[C1,C2,K2,3] + f[C2,C1,K1,3]
   + f[C2,C1,K2,3] <= 3.632
 cap_limit_R_in_C2_R1_3: - 2.489 y[R1,3] + f[C2,R1,K1,3] + f[C2,R1,K2,3]
   + f[R1,C2,K1,3] + f[R1,C2,K2,3] <= 0
 cap_limit_R_in_C2_R2_3: - 3.43 y[R2,3] + f[C2,R2,K1,3] + f[C2,R2,K2,3]
   + f[R2,C2,K1,3] + f[R2,C2,K2,3] <= 0
 cap_limit_R_out_R1_0_3: - 28.689 y[R1,3] + f[0,R1,K1,3] + f[0,R1,K2,3]
   + f[R1,0,K1,3] + f[R1,0,K2,3] <= 0
 cap_limit_R_out_R1_C1_3: - 1.967 y[R1,3] + f[C1,R1,K1,3] + f[C1,R1,K2,3]
   + f[R1,C1,K1,3] + f[R1,C1,K2,3] <= 0
 cap_limit_R_out_R1_C2_3: - 2.489 y[R1,3] + f[C2,R1,K1,3] + f[C2,R1,K2,3]
   + f[R1,C2,K1,3] + f[R1,C2,K2,3] <= 0
 cap_limit_R_out_R1_R2_3: - 6.719 y[R1,3] + f[R1,R2,K1,3] + f[R1,R2,K2,3]
   + f[R2,R1,K1,3] + f[R2,R1,K2,3] <= 0
 cap_limit_R_out_R2_0_3: - 7.706 y[R2,3] + f[0,R2,K1,3] + f[0,R2,K2,3]
   + f[R2,0,K1,3] + f[R2,0,K2,3] <= 0
 cap_limit_R_out_R2_C1_3: - 8.302 y[R2,3] + f[C1,R2,K1,3] + f[C1,R2,K2,3]
   + f[R2,C1,K1,3] + f[R2,C1,K2,3] <= 0
 cap_limit_R_out_R2_C2_3: - 3.43 y[R2,3] + f[C2,R2,K1,3] + f[C2,R2,K2,3]
   + f[R2,C2,K1,3] + f[R2,C2,K2,3] <= 0
 cap_limit_R_out_R2_R1_3: - 6.719 y[R2,3] + f[R1,R2,K1,3] + f[R1,R2,K2,3]
   + f[R2,R1,K1,3] + f[R2,R1,K2,3] <= 0
 cap_limit_N_0_C1_4: f[0,C1,K1,4] + f[0,C1,K2,4] + f[C1,0,K1,4]
   + f[C1,0,K2,4] <= 2.55
 cap_limit_N_0_C2_4: f[0,C2,K1,4] + f[0,C2,K2,4] + f[C2,0,K1,4]
   + f[C2,0,K2,4] <= 4.913
 cap_limit_R_in_0_R1_4: - 28.689 y[R1,4] + f[0,R1,K1,4] + f[0,R1,K2,4]
   + f[R1,0,K1,4] + f[R1,0,K2,4] <= 0
 cap_limit_R_in_0_R2_4: - 7.706 y[R2,4] + f[0,R2,K1,4] + f[0,R2,K2,4]
   + f[R2,0,K1,4] + f[R2,0,K2,4] <= 0
 cap_limit_N_C1_0_4: f[0,C1,K1,4] + f[0,C1,K2,4] + f[C1,0,K1,4]
   + f[C1,0,K2,4] <= 2.55
 cap_limit_N_C1_C2_4: f[C1,C2,K1,4] + f[C1,C2,K2,4] + f[C2,C1,K1,4]
   + f[C2,C1,K2,4] <= 3.632
 cap_limit_R_in_C1_R1_4: - 1.967 y[R1,4] + f[C1,R1,K1,4] + f[C1,R1,K2,4]
   + f[R1,C1,K1,4] + f[R1,C1,K2,4] <= 0
 cap_limit_R_in_C1_R2_4: - 8.302 y[R2,4] + f[C1,R2,K1,4] + f[C1,R2,K2,4]
   + f[R2,C1,K1,4] + f[R2,C1,K2,4] <= 0
 cap_limit_N_C2_0_4: f[0,C2,K1,4] + f[0,C2,K2,4] + f[C2,0,K1,4]
   + f[C2,0,K2,4] <= 4.913
 cap_limit_N_C2_C1_4: f[C1,C2,K1,4] + f[C1,C2,K2,4] + f[C2,C1,K1,4]
   + f[C2,C1,K2,4] <= 3.632
 cap_limit_R_in_C2_R1_4: - 2.489 y[R1,4] + f[C2,R1,K1,4] + f[C2,R1,K2,4]
   + f[R1,C2,K1,4] + f[R1,C2,K2,4] <= 0
 cap_limit_R_in_C2_R2_4: - 3.43 y[R2,4] + f[C2,R2,K1,4] + f[C2,R2,K2,4]
   + f[R2,C2,K1,4] + f[R2,C2,K2,4] <= 0
 cap_limit_R_out_R1_0_4: - 28.689 y[R1,4] + f[0,R1,K1,4] + f[0,R1,K2,4]
   + f[R1,0,K1,4] + f[R1,0,K2,4] <= 0
 cap_limit_R_out_R1_C1_4: - 1.967 y[R1,4] + f[C1,R1,K1,4] + f[C1,R1,K2,4]
   + f[R1,C1,K1,4] + f[R1,C1,K2,4] <= 0
 cap_limit_R_out_R1_C2_4: - 2.489 y[R1,4] + f[C2,R1,K1,4] + f[C2,R1,K2,4]
   + f[R1,C2,K1,4] + f[R1,C2,K2,4] <= 0
 cap_limit_R_out_R1_R2_4: - 6.719 y[R1,4] + f[R1,R2,K1,4] + f[R1,R2,K2,4]
   + f[R2,R1,K1,4] + f[R2,R1,K2,4] <= 0
 cap_limit_R_out_R2_0_4: - 7.706 y[R2,4] + f[0,R2,K1,4] + f[0,R2,K2,4]
   + f[R2,0,K1,4] + f[R2,0,K2,4] <= 0
 cap_limit_R_out_R2_C1_4: - 8.302 y[R2,4] + f[C1,R2,K1,4] + f[C1,R2,K2,4]
   + f[R2,C1,K1,4] + f[R2,C1,K2,4] <= 0
 cap_limit_R_out_R2_C2_4: - 3.43 y[R2,4] + f[C2,R2,K1,4] + f[C2,R2,K2,4]
   + f[R2,C2,K1,4] + f[R2,C2,K2,4] <= 0
 cap_limit_R_out_R2_R1_4: - 6.719 y[R2,4] + f[R1,R2,K1,4] + f[R1,R2,K2,4]
   + f[R2,R1,K1,4] + f[R2,R1,K2,4] <= 0
 cap_limit_N_0_C1_5: f[0,C1,K1,5] + f[0,C1,K2,5] + f[C1,0,K1,5]
   + f[C1,0,K2,5] <= 2.55
 cap_limit_N_0_C2_5: f[0,C2,K1,5] + f[0,C2,K2,5] + f[C2,0,K1,5]
   + f[C2,0,K2,5] <= 4.913
 cap_limit_R_in_0_R1_5: - 28.689 y[R1,5] + f[0,R1,K1,5] + f[0,R1,K2,5]
   + f[R1,0,K1,5] + f[R1,0,K2,5] <= 0
 cap_limit_R_in_0_R2_5: - 7.706 y[R2,5] + f[0,R2,K1,5] + f[0,R2,K2,5]
   + f[R2,0,K1,5] + f[R2,0,K2,5] <= 0
 cap_limit_N_C1_0_5: f[0,C1,K1,5] + f[0,C1,K2,5] + f[C1,0,K1,5]
   + f[C1,0,K2,5] <= 2.55
 cap_limit_N_C1_C2_5: f[C1,C2,K1,5] + f[C1,C2,K2,5] + f[C2,C1,K1,5]
   + f[C2,C1,K2,5] <= 3.632
 cap_limit_R_in_C1_R1_5: - 1.967 y[R1,5] + f[C1,R1,K1,5] + f[C1,R1,K2,5]
   + f[R1,C1,K1,5] + f[R1,C1,K2,5] <= 0
 cap_limit_R_in_C1_R2_5: - 8.302 y[R2,5] + f[C1,R2,K1,5] + f[C1,R2,K2,5]
   + f[R2,C1,K1,5] + f[R2,C1,K2,5] <= 0
 cap_limit_N_C2_0_5: f[0,C2,K1,5] + f[0,C2,K2,5] + f[C2,0,K1,5]
   + f[C2,0,K2,5] <= 4.913
 cap_limit_N_C2_C1_5: f[C1,C2,K1,5] + f[C1,C2,K2,5] + f[C2,C1,K1,5]
   + f[C2,C1,K2,5] <= 3.632
 cap_limit_R_in_C2_R1_5: - 2.489 y[R1,5] + f[C2,R1,K1,5] + f[C2,R1,K2,5]
   + f[R1,C2,K1,5] + f[R1,C2,K2,5] <= 0
 cap_limit_R_in_C2_R2_5: - 3.43 y[R2,5] + f[C2,R2,K1,5] + f[C2,R2,K2,5]
   + f[R2,C2,K1,5] + f[R2,C2,K2,5] <= 0
 cap_limit_R_out_R1_0_5: - 28.689 y[R1,5] + f[0,R1,K1,5] + f[0,R1,K2,5]
   + f[R1,0,K1,5] + f[R1,0,K2,5] <= 0
 cap_limit_R_out_R1_C1_5: - 1.967 y[R1,5] + f[C1,R1,K1,5] + f[C1,R1,K2,5]
   + f[R1,C1,K1,5] + f[R1,C1,K2,5] <= 0
 cap_limit_R_out_R1_C2_5: - 2.489 y[R1,5] + f[C2,R1,K1,5] + f[C2,R1,K2,5]
   + f[R1,C2,K1,5] + f[R1,C2,K2,5] <= 0
 cap_limit_R_out_R1_R2_5: - 6.719 y[R1,5] + f[R1,R2,K1,5] + f[R1,R2,K2,5]
   + f[R2,R1,K1,5] + f[R2,R1,K2,5] <= 0
 cap_limit_R_out_R2_0_5: - 7.706 y[R2,5] + f[0,R2,K1,5] + f[0,R2,K2,5]
   + f[R2,0,K1,5] + f[R2,0,K2,5] <= 0
 cap_limit_R_out_R2_C1_5: - 8.302 y[R2,5] + f[C1,R2,K1,5] + f[C1,R2,K2,5]
   + f[R2,C1,K1,5] + f[R2,C1,K2,5] <= 0
 cap_limit_R_out_R2_C2_5: - 3.43 y[R2,5] + f[C2,R2,K1,5] + f[C2,R2,K2,5]
   + f[R2,C2,K1,5] + f[R2,C2,K2,5] <= 0
 cap_limit_R_out_R2_R1_5: - 6.719 y[R2,5] + f[R1,R2,K1,5] + f[R1,R2,K2,5]
   + f[R2,R1,K1,5] + f[R2,R1,K2,5] <= 0
 cap_limit_N_0_C1_6: f[0,C1,K1,6] + f[0,C1,K2,6] + f[C1,0,K1,6]
   + f[C1,0,K2,6] <= 2.55
 cap_limit_N_0_C2_6: f[0,C2,K1,6] + f[0,C2,K2,6] + f[C2,0,K1,6]
   + f[C2,0,K2,6] <= 4.913
 cap_limit_R_in_0_R1_6: - 28.689 y[R1,6] + f[0,R1,K1,6] + f[0,R1,K2,6]
   + f[R1,0,K1,6] + f[R1,0,K2,6] <= 0
 cap_limit_R_in_0_R2_6: - 7.706 y[R2,6] + f[0,R2,K1,6] + f[0,R2,K2,6]
   + f[R2,0,K1,6] + f[R2,0,K2,6] <= 0
 cap_limit_N_C1_0_6: f[0,C1,K1,6] + f[0,C1,K2,6] + f[C1,0,K1,6]
   + f[C1,0,K2,6] <= 2.55
 cap_limit_N_C1_C2_6: f[C1,C2,K1,6] + f[C1,C2,K2,6] + f[C2,C1,K1,6]
   + f[C2,C1,K2,6] <= 3.632
 cap_limit_R_in_C1_R1_6: - 1.967 y[R1,6] + f[C1,R1,K1,6] + f[C1,R1,K2,6]
   + f[R1,C1,K1,6] + f[R1,C1,K2,6] <= 0
 cap_limit_R_in_C1_R2_6: - 8.302 y[R2,6] + f[C1,R2,K1,6] + f[C1,R2,K2,6]
   + f[R2,C1,K1,6] + f[R2,C1,K2,6] <= 0
 cap_limit_N_C2_0_6: f[0,C2,K1,6] + f[0,C2,K2,6] + f[C2,0,K1,6]
   + f[C2,0,K2,6] <= 4.913
 cap_limit_N_C2_C1_6: f[C1,C2,K1,6] + f[C1,C2,K2,6] + f[C2,C1,K1,6]
   + f[C2,C1,K2,6] <= 3.632
 cap_limit_R_in_C2_R1_6: - 2.489 y[R1,6] + f[C2,R1,K1,6] + f[C2,R1,K2,6]
   + f[R1,C2,K1,6] + f[R1,C2,K2,6] <= 0
 cap_limit_R_in_C2_R2_6: - 3.43 y[R2,6] + f[C2,R2,K1,6] + f[C2,R2,K2,6]
   + f[R2,C2,K1,6] + f[R2,C2,K2,6] <= 0
 cap_limit_R_out_R1_0_6: - 28.689 y[R1,6] + f[0,R1,K1,6] + f[0,R1,K2,6]
   + f[R1,0,K1,6] + f[R1,0,K2,6] <= 0
 cap_limit_R_out_R1_C1_6: - 1.967 y[R1,6] + f[C1,R1,K1,6] + f[C1,R1,K2,6]
   + f[R1,C1,K1,6] + f[R1,C1,K2,6] <= 0
 cap_limit_R_out_R1_C2_6: - 2.489 y[R1,6] + f[C2,R1,K1,6] + f[C2,R1,K2,6]
   + f[R1,C2,K1,6] + f[R1,C2,K2,6] <= 0
 cap_limit_R_out_R1_R2_6: - 6.719 y[R1,6] + f[R1,R2,K1,6] + f[R1,R2,K2,6]
   + f[R2,R1,K1,6] + f[R2,R1,K2,6] <= 0
 cap_limit_R_out_R2_0_6: - 7.706 y[R2,6] + f[0,R2,K1,6] + f[0,R2,K2,6]
   + f[R2,0,K1,6] + f[R2,0,K2,6] <= 0
 cap_limit_R_out_R2_C1_6: - 8.302 y[R2,6] + f[C1,R2,K1,6] + f[C1,R2,K2,6]
   + f[R2,C1,K1,6] + f[R2,C1,K2,6] <= 0
 cap_limit_R_out_R2_C2_6: - 3.43 y[R2,6] + f[C2,R2,K1,6] + f[C2,R2,K2,6]
   + f[R2,C2,K1,6] + f[R2,C2,K2,6] <= 0
 cap_limit_R_out_R2_R1_6: - 6.719 y[R2,6] + f[R1,R2,K1,6] + f[R1,R2,K2,6]
   + f[R2,R1,K1,6] + f[R2,R1,K2,6] <= 0
 cap_limit_N_0_C1_7: f[0,C1,K1,7] + f[0,C1,K2,7] + f[C1,0,K1,7]
   + f[C1,0,K2,7] <= 2.55
 cap_limit_N_0_C2_7: f[0,C2,K1,7] + f[0,C2,K2,7] + f[C2,0,K1,7]
   + f[C2,0,K2,7] <= 4.913
 cap_limit_R_in_0_R1_7: - 28.689 y[R1,7] + f[0,R1,K1,7] + f[0,R1,K2,7]
   + f[R1,0,K1,7] + f[R1,0,K2,7] <= 0
 cap_limit_R_in_0_R2_7: - 7.706 y[R2,7] + f[0,R2,K1,7] + f[0,R2,K2,7]
   + f[R2,0,K1,7] + f[R2,0,K2,7] <= 0
 cap_limit_N_C1_0_7: f[0,C1,K1,7] + f[0,C1,K2,7] + f[C1,0,K1,7]
   + f[C1,0,K2,7] <= 2.55
 cap_limit_N_C1_C2_7: f[C1,C2,K1,7] + f[C1,C2,K2,7] + f[C2,C1,K1,7]
   + f[C2,C1,K2,7] <= 3.632
 cap_limit_R_in_C1_R1_7: - 1.967 y[R1,7] + f[C1,R1,K1,7] + f[C1,R1,K2,7]
   + f[R1,C1,K1,7] + f[R1,C1,K2,7] <= 0
 cap_limit_R_in_C1_R2_7: - 8.302 y[R2,7] + f[C1,R2,K1,7] + f[C1,R2,K2,7]
   + f[R2,C1,K1,7] + f[R2,C1,K2,7] <= 0
 cap_limit_N_C2_0_7: f[0,C2,K1,7] + f[0,C2,K2,7] + f[C2,0,K1,7]
   + f[C2,0,K2,7] <= 4.913
 cap_limit_N_C2_C1_7: f[C1,C2,K1,7] + f[C1,C2,K2,7] + f[C2,C1,K1,7]
   + f[C2,C1,K2,7] <= 3.632
 cap_limit_R_in_C2_R1_7: - 2.489 y[R1,7] + f[C2,R1,K1,7] + f[C2,R1,K2,7]
   + f[R1,C2,K1,7] + f[R1,C2,K2,7] <= 0
 cap_limit_R_in_C2_R2_7: - 3.43 y[R2,7] + f[C2,R2,K1,7] + f[C2,R2,K2,7]
   + f[R2,C2,K1,7] + f[R2,C2,K2,7] <= 0
 cap_limit_R_out_R1_0_7: - 28.689 y[R1,7] + f[0,R1,K1,7] + f[0,R1,K2,7]
   + f[R1,0,K1,7] + f[R1,0,K2,7] <= 0
 cap_limit_R_out_R1_C1_7: - 1.967 y[R1,7] + f[C1,R1,K1,7] + f[C1,R1,K2,7]
   + f[R1,C1,K1,7] + f[R1,C1,K2,7] <= 0
 cap_limit_R_out_R1_C2_7: - 2.489 y[R1,7] + f[C2,R1,K1,7] + f[C2,R1,K2,7]
   + f[R1,C2,K1,7] + f[R1,C2,K2,7] <= 0
 cap_limit_R_out_R1_R2_7: - 6.719 y[R1,7] + f[R1,R2,K1,7] + f[R1,R2,K2,7]
   + f[R2,R1,K1,7] + f[R2,R1,K2,7] <= 0
 cap_limit_R_out_R2_0_7: - 7.706 y[R2,7] + f[0,R2,K1,7] + f[0,R2,K2,7]
   + f[R2,0,K1,7] + f[R2,0,K2,7] <= 0
 cap_limit_R_out_R2_C1_7: - 8.302 y[R2,7] + f[C1,R2,K1,7] + f[C1,R2,K2,7]
   + f[R2,C1,K1,7] + f[R2,C1,K2,7] <= 0
 cap_limit_R_out_R2_C2_7: - 3.43 y[R2,7] + f[C2,R2,K1,7] + f[C2,R2,K2,7]
   + f[R2,C2,K1,7] + f[R2,C2,K2,7] <= 0
 cap_limit_R_out_R2_R1_7: - 6.719 y[R2,7] + f[R1,R2,K1,7] + f[R1,R2,K2,7]
   + f[R2,R1,K1,7] + f[R2,R1,K2,7] <= 0
 cap_limit_N_0_C1_8: f[0,C1,K1,8] + f[0,C1,K2,8] + f[C1,0,K1,8]
   + f[C1,0,K2,8] <= 2.55
 cap_limit_N_0_C2_8: f[0,C2,K1,8] + f[0,C2,K2,8] + f[C2,0,K1,8]
   + f[C2,0,K2,8] <= 4.913
 cap_limit_R_in_0_R1_8: - 28.689 y[R1,8] + f[0,R1,K1,8] + f[0,R1,K2,8]
   + f[R1,0,K1,8] + f[R1,0,K2,8] <= 0
 cap_limit_R_in_0_R2_8: - 7.706 y[R2,8] + f[0,R2,K1,8] + f[0,R2,K2,8]
   + f[R2,0,K1,8] + f[R2,0,K2,8] <= 0
 cap_limit_N_C1_0_8: f[0,C1,K1,8] + f[0,C1,K2,8] + f[C1,0,K1,8]
   + f[C1,0,K2,8] <= 2.55
 cap_limit_N_C1_C2_8: f[C1,C2,K1,8] + f[C1,C2,K2,8] + f[C2,C1,K1,8]
   + f[C2,C1,K2,8] <= 3.632
 cap_limit_R_in_C1_R1_8: - 1.967 y[R1,8] + f[C1,R1,K1,8] + f[C1,R1,K2,8]
   + f[R1,C1,K1,8] + f[R1,C1,K2,8] <= 0
 cap_limit_R_in_C1_R2_8: - 8.302 y[R2,8] + f[C1,R2,K1,8] + f[C1,R2,K2,8]
   + f[R2,C1,K1,8] + f[R2,C1,K2,8] <= 0
 cap_limit_N_C2_0_8: f[0,C2,K1,8] + f[0,C2,K2,8] + f[C2,0,K1,8]
   + f[C2,0,K2,8] <= 4.913
 cap_limit_N_C2_C1_8: f[C1,C2,K1,8] + f[C1,C2,K2,8] + f[C2,C1,K1,8]
   + f[C2,C1,K2,8] <= 3.632
 cap_limit_R_in_C2_R1_8: - 2.489 y[R1,8] + f[C2,R1,K1,8] + f[C2,R1,K2,8]
   + f[R1,C2,K1,8] + f[R1,C2,K2,8] <= 0
 cap_limit_R_in_C2_R2_8: - 3.43 y[R2,8] + f[C2,R2,K1,8] + f[C2,R2,K2,8]
   + f[R2,C2,K1,8] + f[R2,C2,K2,8] <= 0
 cap_limit_R_out_R1_0_8: - 28.689 y[R1,8] + f[0,R1,K1,8] + f[0,R1,K2,8]
   + f[R1,0,K1,8] + f[R1,0,K2,8] <= 0
 cap_limit_R_out_R1_C1_8: - 1.967 y[R1,8] + f[C1,R1,K1,8] + f[C1,R1,K2,8]
   + f[R1,C1,K1,8] + f[R1,C1,K2,8] <= 0
 cap_limit_R_out_R1_C2_8: - 2.489 y[R1,8] + f[C2,R1,K1,8] + f[C2,R1,K2,8]
   + f[R1,C2,K1,8] + f[R1,C2,K2,8] <= 0
 cap_limit_R_out_R1_R2_8: - 6.719 y[R1,8] + f[R1,R2,K1,8] + f[R1,R2,K2,8]
   + f[R2,R1,K1,8] + f[R2,R1,K2,8] <= 0
 cap_limit_R_out_R2_0_8: - 7.706 y[R2,8] + f[0,R2,K1,8] + f[0,R2,K2,8]
   + f[R2,0,K1,8] + f[R2,0,K2,8] <= 0
 cap_limit_R_out_R2_C1_8: - 8.302 y[R2,8] + f[C1,R2,K1,8] + f[C1,R2,K2,8]
   + f[R2,C1,K1,8] + f[R2,C1,K2,8] <= 0
 cap_limit_R_out_R2_C2_8: - 3.43 y[R2,8] + f[C2,R2,K1,8] + f[C2,R2,K2,8]
   + f[R2,C2,K1,8] + f[R2,C2,K2,8] <= 0
 cap_limit_R_out_R2_R1_8: - 6.719 y[R2,8] + f[R1,R2,K1,8] + f[R1,R2,K2,8]
   + f[R2,R1,K1,8] + f[R2,R1,K2,8] <= 0
 cap_limit_N_0_C1_9: f[0,C1,K1,9] + f[0,C1,K2,9] + f[C1,0,K1,9]
   + f[C1,0,K2,9] <= 2.55
 cap_limit_N_0_C2_9: f[0,C2,K1,9] + f[0,C2,K2,9] + f[C2,0,K1,9]
   + f[C2,0,K2,9] <= 4.913
 cap_limit_R_in_0_R1_9: - 28.689 y[R1,9] + f[0,R1,K1,9] + f[0,R1,K2,9]
   + f[R1,0,K1,9] + f[R1,0,K2,9] <= 0
 cap_limit_R_in_0_R2_9: - 7.706 y[R2,9] + f[0,R2,K1,9] + f[0,R2,K2,9]
   + f[R2,0,K1,9] + f[R2,0,K2,9] <= 0
 cap_limit_N_C1_0_9: f[0,C1,K1,9] + f[0,C1,K2,9] + f[C1,0,K1,9]
   + f[C1,0,K2,9] <= 2.55
 cap_limit_N_C1_C2_9: f[C1,C2,K1,9] + f[C1,C2,K2,9] + f[C2,C1,K1,9]
   + f[C2,C1,K2,9] <= 3.632
 cap_limit_R_in_C1_R1_9: - 1.967 y[R1,9] + f[C1,R1,K1,9] + f[C1,R1,K2,9]
   + f[R1,C1,K1,9] + f[R1,C1,K2,9] <= 0
 cap_limit_R_in_C1_R2_9: - 8.302 y[R2,9] + f[C1,R2,K1,9] + f[C1,R2,K2,9]
   + f[R2,C1,K1,9] + f[R2,C1,K2,9] <= 0
 cap_limit_N_C2_0_9: f[0,C2,K1,9] + f[0,C2,K2,9] + f[C2,0,K1,9]
   + f[C2,0,K2,9] <= 4.913
 cap_limit_N_C2_C1_9: f[C1,C2,K1,9] + f[C1,C2,K2,9] + f[C2,C1,K1,9]
   + f[C2,C1,K2,9] <= 3.632
 cap_limit_R_in_C2_R1_9: - 2.489 y[R1,9] + f[C2,R1,K1,9] + f[C2,R1,K2,9]
   + f[R1,C2,K1,9] + f[R1,C2,K2,9] <= 0
 cap_limit_R_in_C2_R2_9: - 3.43 y[R2,9] + f[C2,R2,K1,9] + f[C2,R2,K2,9]
   + f[R2,C2,K1,9] + f[R2,C2,K2,9] <= 0
 cap_limit_R_out_R1_0_9: - 28.689 y[R1,9] + f[0,R1,K1,9] + f[0,R1,K2,9]
   + f[R1,0,K1,9] + f[R1,0,K2,9] <= 0
 cap_limit_R_out_R1_C1_9: - 1.967 y[R1,9] + f[C1,R1,K1,9] + f[C1,R1,K2,9]
   + f[R1,C1,K1,9] + f[R1,C1,K2,9] <= 0
 cap_limit_R_out_R1_C2_9: - 2.489 y[R1,9] + f[C2,R1,K1,9] + f[C2,R1,K2,9]
   + f[R1,C2,K1,9] + f[R1,C2,K2,9] <= 0
 cap_limit_R_out_R1_R2_9: - 6.719 y[R1,9] + f[R1,R2,K1,9] + f[R1,R2,K2,9]
   + f[R2,R1,K1,9] + f[R2,R1,K2,9] <= 0
 cap_limit_R_out_R2_0_9: - 7.706 y[R2,9] + f[0,R2,K1,9] + f[0,R2,K2,9]
   + f[R2,0,K1,9] + f[R2,0,K2,9] <= 0
 cap_limit_R_out_R2_C1_9: - 8.302 y[R2,9] + f[C1,R2,K1,9] + f[C1,R2,K2,9]
   + f[R2,C1,K1,9] + f[R2,C1,K2,9] <= 0
 cap_limit_R_out_R2_C2_9: - 3.43 y[R2,9] + f[C2,R2,K1,9] + f[C2,R2,K2,9]
   + f[R2,C2,K1,9] + f[R2,C2,K2,9] <= 0
 cap_limit_R_out_R2_R1_9: - 6.719 y[R2,9] + f[R1,R2,K1,9] + f[R1,R2,K2,9]
   + f[R2,R1,K1,9] + f[R2,R1,K2,9] <= 0
 cap_limit_N_0_C1_10: f[0,C1,K1,10] + f[0,C1,K2,10] + f[C1,0,K1,10]
   + f[C1,0,K2,10] <= 2.55
 cap_limit_N_0_C2_10: f[0,C2,K1,10] + f[0,C2,K2,10] + f[C2,0,K1,10]
   + f[C2,0,K2,10] <= 4.913
 cap_limit_R_in_0_R1_10: - 28.689 y[R1,10] + f[0,R1,K1,10] + f[0,R1,K2,10]
   + f[R1,0,K1,10] + f[R1,0,K2,10] <= 0
 cap_limit_R_in_0_R2_10: - 7.706 y[R2,10] + f[0,R2,K1,10] + f[0,R2,K2,10]
   + f[R2,0,K1,10] + f[R2,0,K2,10] <= 0
 cap_limit_N_C1_0_10: f[0,C1,K1,10] + f[0,C1,K2,10] + f[C1,0,K1,10]
   + f[C1,0,K2,10] <= 2.55
 cap_limit_N_C1_C2_10: f[C1,C2,K1,10] + f[C1,C2,K2,10] + f[C2,C1,K1,10]
   + f[C2,C1,K2,10] <= 3.632
 cap_limit_R_in_C1_R1_10: - 1.967 y[R1,10] + f[C1,R1,K1,10]
   + f[C1,R1,K2,10] + f[R1,C1,K1,10] + f[R1,C1,K2,10] <= 0
 cap_limit_R_in_C1_R2_10: - 8.302 y[R2,10] + f[C1,R2,K1,10]
   + f[C1,R2,K2,10] + f[R2,C1,K1,10] + f[R2,C1,K2,10] <= 0
 cap_limit_N_C2_0_10: f[0,C2,K1,10] + f[0,C2,K2,10] + f[C2,0,K1,10]
   + f[C2,0,K2,10] <= 4.913
 cap_limit_N_C2_C1_10: f[C1,C2,K1,10] + f[C1,C2,K2,10] + f[C2,C1,K1,10]
   + f[C2,C1,K2,10] <= 3.632
 cap_limit_R_in_C2_R1_10: - 2.489 y[R1,10] + f[C2,R1,K1,10]
   + f[C2,R1,K2,10] + f[R1,C2,K1,10] + f[R1,C2,K2,10] <= 0
 cap_limit_R_in_C2_R2_10: - 3.43 y[R2,10] + f[C2,R2,K1,10] + f[C2,R2,K2,10]
   + f[R2,C2,K1,10] + f[R2,C2,K2,10] <= 0
 cap_limit_R_out_R1_0_10: - 28.689 y[R1,10] + f[0,R1,K1,10] + f[0,R1,K2,10]
   + f[R1,0,K1,10] + f[R1,0,K2,10] <= 0
 cap_limit_R_out_R1_C1_10: - 1.967 y[R1,10] + f[C1,R1,K1,10]
   + f[C1,R1,K2,10] + f[R1,C1,K1,10] + f[R1,C1,K2,10] <= 0
 cap_limit_R_out_R1_C2_10: - 2.489 y[R1,10] + f[C2,R1,K1,10]
   + f[C2,R1,K2,10] + f[R1,C2,K1,10] + f[R1,C2,K2,10] <= 0
 cap_limit_R_out_R1_R2_10: - 6.719 y[R1,10] + f[R1,R2,K1,10]
   + f[R1,R2,K2,10] + f[R2,R1,K1,10] + f[R2,R1,K2,10] <= 0
 cap_limit_R_out_R2_0_10: - 7.706 y[R2,10] + f[0,R2,K1,10] + f[0,R2,K2,10]
   + f[R2,0,K1,10] + f[R2,0,K2,10] <= 0
 cap_limit_R_out_R2_C1_10: - 8.302 y[R2,10] + f[C1,R2,K1,10]
   + f[C1,R2,K2,10] + f[R2,C1,K1,10] + f[R2,C1,K2,10] <= 0
 cap_limit_R_out_R2_C2_10: - 3.43 y[R2,10] + f[C2,R2,K1,10]
   + f[C2,R2,K2,10] + f[R2,C2,K1,10] + f[R2,C2,K2,10] <= 0
 cap_limit_R_out_R2_R1_10: - 6.719 y[R2,10] + f[R1,R2,K1,10]
   + f[R1,R2,K2,10] + f[R2,R1,K1,10] + f[R2,R1,K2,10] <= 0
 cap_limit_N_0_C1_11: f[0,C1,K1,11] + f[0,C1,K2,11] + f[C1,0,K1,11]
   + f[C1,0,K2,11] <= 2.55
 cap_limit_N_0_C2_11: f[0,C2,K1,11] + f[0,C2,K2,11] + f[C2,0,K1,11]
   + f[C2,0,K2,11] <= 4.913
 cap_limit_R_in_0_R1_11: - 28.689 y[R1,11] + f[0,R1,K1,11] + f[0,R1,K2,11]
   + f[R1,0,K1,11] + f[R1,0,K2,11] <= 0
 cap_limit_R_in_0_R2_11: - 7.706 y[R2,11] + f[0,R2,K1,11] + f[0,R2,K2,11]
   + f[R2,0,K1,11] + f[R2,0,K2,11] <= 0
 cap_limit_N_C1_0_11: f[0,C1,K1,11] + f[0,C1,K2,11] + f[C1,0,K1,11]
   + f[C1,0,K2,11] <= 2.55
 cap_limit_N_C1_C2_11: f[C1,C2,K1,11] + f[C1,C2,K2,11] + f[C2,C1,K1,11]
   + f[C2,C1,K2,11] <= 3.632
 cap_limit_R_in_C1_R1_11: - 1.967 y[R1,11] + f[C1,R1,K1,11]
   + f[C1,R1,K2,11] + f[R1,C1,K1,11] + f[R1,C1,K2,11] <= 0
 cap_limit_R_in_C1_R2_11: - 8.302 y[R2,11] + f[C1,R2,K1,11]
   + f[C1,R2,K2,11] + f[R2,C1,K1,11] + f[R2,C1,K2,11] <= 0
 cap_limit_N_C2_0_11: f[0,C2,K1,11] + f[0,C2,K2,11] + f[C2,0,K1,11]
   + f[C2,0,K2,11] <= 4.913
 cap_limit_N_C2_C1_11: f[C1,C2,K1,11] + f[C1,C2,K2,11] + f[C2,C1,K1,11]
   + f[C2,C1,K2,11] <= 3.632
 cap_limit_R_in_C2_R1_11: - 2.489 y[R1,11] + f[C2,R1,K1,11]
   + f[C2,R1,K2,11] + f[R1,C2,K1,11] + f[R1,C2,K2,11] <= 0
 cap_limit_R_in_C2_R2_11: - 3.43 y[R2,11] + f[C2,R2,K1,11] + f[C2,R2,K2,11]
   + f[R2,C2,K1,11] + f[R2,C2,K2,11] <= 0
 cap_limit_R_out_R1_0_11: - 28.689 y[R1,11] + f[0,R1,K1,11] + f[0,R1,K2,11]
   + f[R1,0,K1,11] + f[R1,0,K2,11] <= 0
 cap_limit_R_out_R1_C1_11: - 1.967 y[R1,11] + f[C1,R1,K1,11]
   + f[C1,R1,K2,11] + f[R1,C1,K1,11] + f[R1,C1,K2,11] <= 0
 cap_limit_R_out_R1_C2_11: - 2.489 y[R1,11] + f[C2,R1,K1,11]
   + f[C2,R1,K2,11] + f[R1,C2,K1,11] + f[R1,C2,K2,11] <= 0
 cap_limit_R_out_R1_R2_11: - 6.719 y[R1,11] + f[R1,R2,K1,11]
   + f[R1,R2,K2,11] + f[R2,R1,K1,11] + f[R2,R1,K2,11] <= 0
 cap_limit_R_out_R2_0_11: - 7.706 y[R2,11] + f[0,R2,K1,11] + f[0,R2,K2,11]
   + f[R2,0,K1,11] + f[R2,0,K2,11] <= 0
 cap_limit_R_out_R2_C1_11: - 8.302 y[R2,11] + f[C1,R2,K1,11]
   + f[C1,R2,K2,11] + f[R2,C1,K1,11] + f[R2,C1,K2,11] <= 0
 cap_limit_R_out_R2_C2_11: - 3.43 y[R2,11] + f[C2,R2,K1,11]
   + f[C2,R2,K2,11] + f[R2,C2,K1,11] + f[R2,C2,K2,11] <= 0
 cap_limit_R_out_R2_R1_11: - 6.719 y[R2,11] + f[R1,R2,K1,11]
   + f[R1,R2,K2,11] + f[R2,R1,K1,11] + f[R2,R1,K2,11] <= 0
 cap_limit_N_0_C1_12: f[0,C1,K1,12] + f[0,C1,K2,12] + f[C1,0,K1,12]
   + f[C1,0,K2,12] <= 2.55
 cap_limit_N_0_C2_12: f[0,C2,K1,12] + f[0,C2,K2,12] + f[C2,0,K1,12]
   + f[C2,0,K2,12] <= 4.913
 cap_limit_R_in_0_R1_12: - 28.689 y[R1,12] + f[0,R1,K1,12] + f[0,R1,K2,12]
   + f[R1,0,K1,12] + f[R1,0,K2,12] <= 0
 cap_limit_R_in_0_R2_12: - 7.706 y[R2,12] + f[0,R2,K1,12] + f[0,R2,K2,12]
   + f[R2,0,K1,12] + f[R2,0,K2,12] <= 0
 cap_limit_N_C1_0_12: f[0,C1,K1,12] + f[0,C1,K2,12] + f[C1,0,K1,12]
   + f[C1,0,K2,12] <= 2.55
 cap_limit_N_C1_C2_12: f[C1,C2,K1,12] + f[C1,C2,K2,12] + f[C2,C1,K1,12]
   + f[C2,C1,K2,12] <= 3.632
 cap_limit_R_in_C1_R1_12: - 1.967 y[R1,12] + f[C1,R1,K1,12]
   + f[C1,R1,K2,12] + f[R1,C1,K1,12] + f[R1,C1,K2,12] <= 0
 cap_limit_R_in_C1_R2_12: - 8.302 y[R2,12] + f[C1,R2,K1,12]
   + f[C1,R2,K2,12] + f[R2,C1,K1,12] + f[R2,C1,K2,12] <= 0
 cap_limit_N_C2_0_12: f[0,C2,K1,12] + f[0,C2,K2,12] + f[C2,0,K1,12]
   + f[C2,0,K2,12] <= 4.913
 cap_limit_N_C2_C1_12: f[C1,C2,K1,12] + f[C1,C2,K2,12] + f[C2,C1,K1,12]
   + f[C2,C1,K2,12] <= 3.632
 cap_limit_R_in_C2_R1_12: - 2.489 y[R1,12] + f[C2,R1,K1,12]
   + f[C2,R1,K2,12] + f[R1,C2,K1,12] + f[R1,C2,K2,12] <= 0
 cap_limit_R_in_C2_R2_12: - 3.43 y[R2,12] + f[C2,R2,K1,12] + f[C2,R2,K2,12]
   + f[R2,C2,K1,12] + f[R2,C2,K2,12] <= 0
 cap_limit_R_out_R1_0_12: - 28.689 y[R1,12] + f[0,R1,K1,12] + f[0,R1,K2,12]
   + f[R1,0,K1,12] + f[R1,0,K2,12] <= 0
 cap_limit_R_out_R1_C1_12: - 1.967 y[R1,12] + f[C1,R1,K1,12]
   + f[C1,R1,K2,12] + f[R1,C1,K1,12] + f[R1,C1,K2,12] <= 0
 cap_limit_R_out_R1_C2_12: - 2.489 y[R1,12] + f[C2,R1,K1,12]
   + f[C2,R1,K2,12] + f[R1,C2,K1,12] + f[R1,C2,K2,12] <= 0
 cap_limit_R_out_R1_R2_12: - 6.719 y[R1,12] + f[R1,R2,K1,12]
   + f[R1,R2,K2,12] + f[R2,R1,K1,12] + f[R2,R1,K2,12] <= 0
 cap_limit_R_out_R2_0_12: - 7.706 y[R2,12] + f[0,R2,K1,12] + f[0,R2,K2,12]
   + f[R2,0,K1,12] + f[R2,0,K2,12] <= 0
 cap_limit_R_out_R2_C1_12: - 8.302 y[R2,12] + f[C1,R2,K1,12]
   + f[C1,R2,K2,12] + f[R2,C1,K1,12] + f[R2,C1,K2,12] <= 0
 cap_limit_R_out_R2_C2_12: - 3.43 y[R2,12] + f[C2,R2,K1,12]
   + f[C2,R2,K2,12] + f[R2,C2,K1,12] + f[R2,C2,K2,12] <= 0
 cap_limit_R_out_R2_R1_12: - 6.719 y[R2,12] + f[R1,R2,K1,12]
   + f[R1,R2,K2,12] + f[R2,R1,K1,12] + f[R2,R1,K2,12] <= 0
 cap_limit_N_0_C1_13: f[0,C1,K1,13] + f[0,C1,K2,13] + f[C1,0,K1,13]
   + f[C1,0,K2,13] <= 2.55
 cap_limit_N_0_C2_13: f[0,C2,K1,13] + f[0,C2,K2,13] + f[C2,0,K1,13]
   + f[C2,0,K2,13] <= 4.913
 cap_limit_R_in_0_R1_13: - 28.689 y[R1,13] + f[0,R1,K1,13] + f[0,R1,K2,13]
   + f[R1,0,K1,13] + f[R1,0,K2,13] <= 0
 cap_limit_R_in_0_R2_13: - 7.706 y[R2,13] + f[0,R2,K1,13] + f[0,R2,K2,13]
   + f[R2,0,K1,13] + f[R2,0,K2,13] <= 0
 cap_limit_N_C1_0_13: f[0,C1,K1,13] + f[0,C1,K2,13] + f[C1,0,K1,13]
   + f[C1,0,K2,13] <= 2.55
 cap_limit_N_C1_C2_13: f[C1,C2,K1,13] + f[C1,C2,K2,13] + f[C2,C1,K1,13]
   + f[C2,C1,K2,13] <= 3.632
 cap_limit_R_in_C1_R1_13: - 1.967 y[R1,13] + f[C1,R1,K1,13]
   + f[C1,R1,K2,13] + f[R1,C1,K1,13] + f[R1,C1,K2,13] <= 0
 cap_limit_R_in_C1_R2_13: - 8.302 y[R2,13] + f[C1,R2,K1,13]
   + f[C1,R2,K2,13] + f[R2,C1,K1,13] + f[R2,C1,K2,13] <= 0
 cap_limit_N_C2_0_13: f[0,C2,K1,13] + f[0,C2,K2,13] + f[C2,0,K1,13]
   + f[C2,0,K2,13] <= 4.913
 cap_limit_N_C2_C1_13: f[C1,C2,K1,13] + f[C1,C2,K2,13] + f[C2,C1,K1,13]
   + f[C2,C1,K2,13] <= 3.632
 cap_limit_R_in_C2_R1_13: - 2.489 y[R1,13] + f[C2,R1,K1,13]
   + f[C2,R1,K2,13] + f[R1,C2,K1,13] + f[R1,C2,K2,13] <= 0
 cap_limit_R_in_C2_R2_13: - 3.43 y[R2,13] + f[C2,R2,K1,13] + f[C2,R2,K2,13]
   + f[R2,C2,K1,13] + f[R2,C2,K2,13] <= 0
 cap_limit_R_out_R1_0_13: - 28.689 y[R1,13] + f[0,R1,K1,13] + f[0,R1,K2,13]
   + f[R1,0,K1,13] + f[R1,0,K2,13] <= 0
 cap_limit_R_out_R1_C1_13: - 1.967 y[R1,13] + f[C1,R1,K1,13]
   + f[C1,R1,K2,13] + f[R1,C1,K1,13] + f[R1,C1,K2,13] <= 0
 cap_limit_R_out_R1_C2_13: - 2.489 y[R1,13] + f[C2,R1,K1,13]
   + f[C2,R1,K2,13] + f[R1,C2,K1,13] + f[R1,C2,K2,13] <= 0
 cap_limit_R_out_R1_R2_13: - 6.719 y[R1,13] + f[R1,R2,K1,13]
   + f[R1,R2,K2,13] + f[R2,R1,K1,13] + f[R2,R1,K2,13] <= 0
 cap_limit_R_out_R2_0_13: - 7.706 y[R2,13] + f[0,R2,K1,13] + f[0,R2,K2,13]
   + f[R2,0,K1,13] + f[R2,0,K2,13] <= 0
 cap_limit_R_out_R2_C1_13: - 8.302 y[R2,13] + f[C1,R2,K1,13]
   + f[C1,R2,K2,13] + f[R2,C1,K1,13] + f[R2,C1,K2,13] <= 0
 cap_limit_R_out_R2_C2_13: - 3.43 y[R2,13] + f[C2,R2,K1,13]
   + f[C2,R2,K2,13] + f[R2,C2,K1,13] + f[R2,C2,K2,13] <= 0
 cap_limit_R_out_R2_R1_13: - 6.719 y[R2,13] + f[R1,R2,K1,13]
   + f[R1,R2,K2,13] + f[R2,R1,K1,13] + f[R2,R1,K2,13] <= 0
 cap_limit_N_0_C1_14: f[0,C1,K1,14] + f[0,C1,K2,14] + f[C1,0,K1,14]
   + f[C1,0,K2,14] <= 2.55
 cap_limit_N_0_C2_14: f[0,C2,K1,14] + f[0,C2,K2,14] + f[C2,0,K1,14]
   + f[C2,0,K2,14] <= 4.913
 cap_limit_R_in_0_R1_14: - 28.689 y[R1,14] + f[0,R1,K1,14] + f[0,R1,K2,14]
   + f[R1,0,K1,14] + f[R1,0,K2,14] <= 0
 cap_limit_R_in_0_R2_14: - 7.706 y[R2,14] + f[0,R2,K1,14] + f[0,R2,K2,14]
   + f[R2,0,K1,14] + f[R2,0,K2,14] <= 0
 cap_limit_N_C1_0_14: f[0,C1,K1,14] + f[0,C1,K2,14] + f[C1,0,K1,14]
   + f[C1,0,K2,14] <= 2.55
 cap_limit_N_C1_C2_14: f[C1,C2,K1,14] + f[C1,C2,K2,14] + f[C2,C1,K1,14]
   + f[C2,C1,K2,14] <= 3.632
 cap_limit_R_in_C1_R1_14: - 1.967 y[R1,14] + f[C1,R1,K1,14]
   + f[C1,R1,K2,14] + f[R1,C1,K1,14] + f[R1,C1,K2,14] <= 0
 cap_limit_R_in_C1_R2_14: - 8.302 y[R2,14] + f[C1,R2,K1,14]
   + f[C1,R2,K2,14] + f[R2,C1,K1,14] + f[R2,C1,K2,14] <= 0
 cap_limit_N_C2_0_14: f[0,C2,K1,14] + f[0,C2,K2,14] + f[C2,0,K1,14]
   + f[C2,0,K2,14] <= 4.913
 cap_limit_N_C2_C1_14: f[C1,C2,K1,14] + f[C1,C2,K2,14] + f[C2,C1,K1,14]
   + f[C2,C1,K2,14] <= 3.632
 cap_limit_R_in_C2_R1_14: - 2.489 y[R1,14] + f[C2,R1,K1,14]
   + f[C2,R1,K2,14] + f[R1,C2,K1,14] + f[R1,C2,K2,14] <= 0
 cap_limit_R_in_C2_R2_14: - 3.43 y[R2,14] + f[C2,R2,K1,14] + f[C2,R2,K2,14]
   + f[R2,C2,K1,14] + f[R2,C2,K2,14] <= 0
 cap_limit_R_out_R1_0_14: - 28.689 y[R1,14] + f[0,R1,K1,14] + f[0,R1,K2,14]
   + f[R1,0,K1,14] + f[R1,0,K2,14] <= 0
 cap_limit_R_out_R1_C1_14: - 1.967 y[R1,14] + f[C1,R1,K1,14]
   + f[C1,R1,K2,14] + f[R1,C1,K1,14] + f[R1,C1,K2,14] <= 0
 cap_limit_R_out_R1_C2_14: - 2.489 y[R1,14] + f[C2,R1,K1,14]
   + f[C2,R1,K2,14] + f[R1,C2,K1,14] + f[R1,C2,K2,14] <= 0
 cap_limit_R_out_R1_R2_14: - 6.719 y[R1,14] + f[R1,R2,K1,14]
   + f[R1,R2,K2,14] + f[R2,R1,K1,14] + f[R2,R1,K2,14] <= 0
 cap_limit_R_out_R2_0_14: - 7.706 y[R2,14] + f[0,R2,K1,14] + f[0,R2,K2,14]
   + f[R2,0,K1,14] + f[R2,0,K2,14] <= 0
 cap_limit_R_out_R2_C1_14: - 8.302 y[R2,14] + f[C1,R2,K1,14]
   + f[C1,R2,K2,14] + f[R2,C1,K1,14] + f[R2,C1,K2,14] <= 0
 cap_limit_R_out_R2_C2_14: - 3.43 y[R2,14] + f[C2,R2,K1,14]
   + f[C2,R2,K2,14] + f[R2,C2,K1,14] + f[R2,C2,K2,14] <= 0
 cap_limit_R_out_R2_R1_14: - 6.719 y[R2,14] + f[R1,R2,K1,14]
   + f[R1,R2,K2,14] + f[R2,R1,K1,14] + f[R2,R1,K2,14] <= 0
 cap_limit_N_0_C1_15: f[0,C1,K1,15] + f[0,C1,K2,15] + f[C1,0,K1,15]
   + f[C1,0,K2,15] <= 2.55
 cap_limit_N_0_C2_15: f[0,C2,K1,15] + f[0,C2,K2,15] + f[C2,0,K1,15]
   + f[C2,0,K2,15] <= 4.913
 cap_limit_R_in_0_R1_15: - 28.689 y[R1,15] + f[0,R1,K1,15] + f[0,R1,K2,15]
   + f[R1,0,K1,15] + f[R1,0,K2,15] <= 0
 cap_limit_R_in_0_R2_15: - 7.706 y[R2,15] + f[0,R2,K1,15] + f[0,R2,K2,15]
   + f[R2,0,K1,15] + f[R2,0,K2,15] <= 0
 cap_limit_N_C1_0_15: f[0,C1,K1,15] + f[0,C1,K2,15] + f[C1,0,K1,15]
   + f[C1,0,K2,15] <= 2.55
 cap_limit_N_C1_C2_15: f[C1,C2,K1,15] + f[C1,C2,K2,15] + f[C2,C1,K1,15]
   + f[C2,C1,K2,15] <= 3.632
 cap_limit_R_in_C1_R1_15: - 1.967 y[R1,15] + f[C1,R1,K1,15]
   + f[C1,R1,K2,15] + f[R1,C1,K1,15] + f[R1,C1,K2,15] <= 0
 cap_limit_R_in_C1_R2_15: - 8.302 y[R2,15] + f[C1,R2,K1,15]
   + f[C1,R2,K2,15] + f[R2,C1,K1,15] + f[R2,C1,K2,15] <= 0
 cap_limit_N_C2_0_15: f[0,C2,K1,15] + f[0,C2,K2,15] + f[C2,0,K1,15]
   + f[C2,0,K2,15] <= 4.913
 cap_limit_N_C2_C1_15: f[C1,C2,K1,15] + f[C1,C2,K2,15] + f[C2,C1,K1,15]
   + f[C2,C1,K2,15] <= 3.632
 cap_limit_R_in_C2_R1_15: - 2.489 y[R1,15] + f[C2,R1,K1,15]
   + f[C2,R1,K2,15] + f[R1,C2,K1,15] + f[R1,C2,K2,15] <= 0
 cap_limit_R_in_C2_R2_15: - 3.43 y[R2,15] + f[C2,R2,K1,15] + f[C2,R2,K2,15]
   + f[R2,C2,K1,15] + f[R2,C2,K2,15] <= 0
 cap_limit_R_out_R1_0_15: - 28.689 y[R1,15] + f[0,R1,K1,15] + f[0,R1,K2,15]
   + f[R1,0,K1,15] + f[R1,0,K2,15] <= 0
 cap_limit_R_out_R1_C1_15: - 1.967 y[R1,15] + f[C1,R1,K1,15]
   + f[C1,R1,K2,15] + f[R1,C1,K1,15] + f[R1,C1,K2,15] <= 0
 cap_limit_R_out_R1_C2_15: - 2.489 y[R1,15] + f[C2,R1,K1,15]
   + f[C2,R1,K2,15] + f[R1,C2,K1,15] + f[R1,C2,K2,15] <= 0
 cap_limit_R_out_R1_R2_15: - 6.719 y[R1,15] + f[R1,R2,K1,15]
   + f[R1,R2,K2,15] + f[R2,R1,K1,15] + f[R2,R1,K2,15] <= 0
 cap_limit_R_out_R2_0_15: - 7.706 y[R2,15] + f[0,R2,K1,15] + f[0,R2,K2,15]
   + f[R2,0,K1,15] + f[R2,0,K2,15] <= 0
 cap_limit_R_out_R2_C1_15: - 8.302 y[R2,15] + f[C1,R2,K1,15]
   + f[C1,R2,K2,15] + f[R2,C1,K1,15] + f[R2,C1,K2,15] <= 0
 cap_limit_R_out_R2_C2_15: - 3.43 y[R2,15] + f[C2,R2,K1,15]
   + f[C2,R2,K2,15] + f[R2,C2,K1,15] + f[R2,C2,K2,15] <= 0
 cap_limit_R_out_R2_R1_15: - 6.719 y[R2,15] + f[R1,R2,K1,15]
   + f[R1,R2,K2,15] + f[R2,R1,K1,15] + f[R2,R1,K2,15] <= 0
 comm_sink_K1_1: f[0,C1,K1,1] - f[C1,0,K1,1] - f[C1,C2,K1,1]
   - f[C1,R1,K1,1] - f[C1,R2,K1,1] + f[C2,C1,K1,1] + f[R1,C1,K1,1]
   + f[R2,C1,K1,1] + [ - v[C1,K1,1] * b[K1,1] ] = 0
 comm_sink_K1_2: f[0,C1,K1,2] - f[C1,0,K1,2] - f[C1,C2,K1,2]
   - f[C1,R1,K1,2] - f[C1,R2,K1,2] + f[C2,C1,K1,2] + f[R1,C1,K1,2]
   + f[R2,C1,K1,2] + [ - v[C1,K1,2] * b[K1,2] ] = 0
 comm_sink_K1_3: f[0,C1,K1,3] - f[C1,0,K1,3] - f[C1,C2,K1,3]
   - f[C1,R1,K1,3] - f[C1,R2,K1,3] + f[C2,C1,K1,3] + f[R1,C1,K1,3]
   + f[R2,C1,K1,3] + [ - v[C1,K1,3] * b[K1,3] ] = 0
 comm_sink_K1_4: f[0,C1,K1,4] - f[C1,0,K1,4] - f[C1,C2,K1,4]
   - f[C1,R1,K1,4] - f[C1,R2,K1,4] + f[C2,C1,K1,4] + f[R1,C1,K1,4]
   + f[R2,C1,K1,4] + [ - v[C1,K1,4] * b[K1,4] ] = 0
 comm_sink_K1_5: f[0,C1,K1,5] - f[C1,0,K1,5] - f[C1,C2,K1,5]
   - f[C1,R1,K1,5] - f[C1,R2,K1,5] + f[C2,C1,K1,5] + f[R1,C1,K1,5]
   + f[R2,C1,K1,5] + [ - v[C1,K1,5] * b[K1,5] ] = 0
 comm_sink_K1_6: f[0,C1,K1,6] - f[C1,0,K1,6] - f[C1,C2,K1,6]
   - f[C1,R1,K1,6] - f[C1,R2,K1,6] + f[C2,C1,K1,6] + f[R1,C1,K1,6]
   + f[R2,C1,K1,6] + [ - v[C1,K1,6] * b[K1,6] ] = 0
 comm_sink_K1_7: f[0,C1,K1,7] - f[C1,0,K1,7] - f[C1,C2,K1,7]
   - f[C1,R1,K1,7] - f[C1,R2,K1,7] + f[C2,C1,K1,7] + f[R1,C1,K1,7]
   + f[R2,C1,K1,7] + [ - v[C1,K1,7] * b[K1,7] ] = 0
 comm_sink_K1_8: f[0,C1,K1,8] - f[C1,0,K1,8] - f[C1,C2,K1,8]
   - f[C1,R1,K1,8] - f[C1,R2,K1,8] + f[C2,C1,K1,8] + f[R1,C1,K1,8]
   + f[R2,C1,K1,8] + [ - v[C1,K1,8] * b[K1,8] ] = 0
 comm_sink_K1_9: f[0,C1,K1,9] - f[C1,0,K1,9] - f[C1,C2,K1,9]
   - f[C1,R1,K1,9] - f[C1,R2,K1,9] + f[C2,C1,K1,9] + f[R1,C1,K1,9]
   + f[R2,C1,K1,9] + [ - v[C1,K1,9] * b[K1,9] ] = 0
 comm_sink_K1_10: f[0,C1,K1,10] - f[C1,0,K1,10] - f[C1,C2,K1,10]
   - f[C1,R1,K1,10] - f[C1,R2,K1,10] + f[C2,C1,K1,10] + f[R1,C1,K1,10]
   + f[R2,C1,K1,10] + [ - v[C1,K1,10] * b[K1,10] ] = 0
 comm_sink_K1_11: f[0,C1,K1,11] - f[C1,0,K1,11] - f[C1,C2,K1,11]
   - f[C1,R1,K1,11] - f[C1,R2,K1,11] + f[C2,C1,K1,11] + f[R1,C1,K1,11]
   + f[R2,C1,K1,11] + [ - v[C1,K1,11] * b[K1,11] ] = 0
 comm_sink_K1_12: f[0,C1,K1,12] - f[C1,0,K1,12] - f[C1,C2,K1,12]
   - f[C1,R1,K1,12] - f[C1,R2,K1,12] + f[C2,C1,K1,12] + f[R1,C1,K1,12]
   + f[R2,C1,K1,12] + [ - v[C1,K1,12] * b[K1,12] ] = 0
 comm_sink_K1_13: f[0,C1,K1,13] - f[C1,0,K1,13] - f[C1,C2,K1,13]
   - f[C1,R1,K1,13] - f[C1,R2,K1,13] + f[C2,C1,K1,13] + f[R1,C1,K1,13]
   + f[R2,C1,K1,13] + [ - v[C1,K1,13] * b[K1,13] ] = 0
 comm_sink_K1_14: f[0,C1,K1,14] - f[C1,0,K1,14] - f[C1,C2,K1,14]
   - f[C1,R1,K1,14] - f[C1,R2,K1,14] + f[C2,C1,K1,14] + f[R1,C1,K1,14]
   + f[R2,C1,K1,14] + [ - v[C1,K1,14] * b[K1,14] ] = 0
 comm_sink_K1_15: f[0,C1,K1,15] - f[C1,0,K1,15] - f[C1,C2,K1,15]
   - f[C1,R1,K1,15] - f[C1,R2,K1,15] + f[C2,C1,K1,15] + f[R1,C1,K1,15]
   + f[R2,C1,K1,15] + [ - v[C1,K1,15] * b[K1,15] ] = 0
 comm_sink_K2_1: f[0,C2,K2,1] + f[C1,C2,K2,1] - f[C2,0,K2,1]
   - f[C2,C1,K2,1] - f[C2,R1,K2,1] - f[C2,R2,K2,1] + f[R1,C2,K2,1]
   + f[R2,C2,K2,1] + [ - v[C2,K2,1] * b[K2,1] ] = 0
 comm_sink_K2_2: f[0,C2,K2,2] + f[C1,C2,K2,2] - f[C2,0,K2,2]
   - f[C2,C1,K2,2] - f[C2,R1,K2,2] - f[C2,R2,K2,2] + f[R1,C2,K2,2]
   + f[R2,C2,K2,2] + [ - v[C2,K2,2] * b[K2,2] ] = 0
 comm_sink_K2_3: f[0,C2,K2,3] + f[C1,C2,K2,3] - f[C2,0,K2,3]
   - f[C2,C1,K2,3] - f[C2,R1,K2,3] - f[C2,R2,K2,3] + f[R1,C2,K2,3]
   + f[R2,C2,K2,3] + [ - v[C2,K2,3] * b[K2,3] ] = 0
 comm_sink_K2_4: f[0,C2,K2,4] + f[C1,C2,K2,4] - f[C2,0,K2,4]
   - f[C2,C1,K2,4] - f[C2,R1,K2,4] - f[C2,R2,K2,4] + f[R1,C2,K2,4]
   + f[R2,C2,K2,4] + [ - v[C2,K2,4] * b[K2,4] ] = 0
 comm_sink_K2_5: f[0,C2,K2,5] + f[C1,C2,K2,5] - f[C2,0,K2,5]
   - f[C2,C1,K2,5] - f[C2,R1,K2,5] - f[C2,R2,K2,5] + f[R1,C2,K2,5]
   + f[R2,C2,K2,5] + [ - v[C2,K2,5] * b[K2,5] ] = 0
 comm_sink_K2_6: f[0,C2,K2,6] + f[C1,C2,K2,6] - f[C2,0,K2,6]
   - f[C2,C1,K2,6] - f[C2,R1,K2,6] - f[C2,R2,K2,6] + f[R1,C2,K2,6]
   + f[R2,C2,K2,6] + [ - v[C2,K2,6] * b[K2,6] ] = 0
 comm_sink_K2_7: f[0,C2,K2,7] + f[C1,C2,K2,7] - f[C2,0,K2,7]
   - f[C2,C1,K2,7] - f[C2,R1,K2,7] - f[C2,R2,K2,7] + f[R1,C2,K2,7]
   + f[R2,C2,K2,7] + [ - v[C2,K2,7] * b[K2,7] ] = 0
 comm_sink_K2_8: f[0,C2,K2,8] + f[C1,C2,K2,8] - f[C2,0,K2,8]
   - f[C2,C1,K2,8] - f[C2,R1,K2,8] - f[C2,R2,K2,8] + f[R1,C2,K2,8]
   + f[R2,C2,K2,8] + [ - v[C2,K2,8] * b[K2,8] ] = 0
 comm_sink_K2_9: f[0,C2,K2,9] + f[C1,C2,K2,9] - f[C2,0,K2,9]
   - f[C2,C1,K2,9] - f[C2,R1,K2,9] - f[C2,R2,K2,9] + f[R1,C2,K2,9]
   + f[R2,C2,K2,9] + [ - v[C2,K2,9] * b[K2,9] ] = 0
 comm_sink_K2_10: f[0,C2,K2,10] + f[C1,C2,K2,10] - f[C2,0,K2,10]
   - f[C2,C1,K2,10] - f[C2,R1,K2,10] - f[C2,R2,K2,10] + f[R1,C2,K2,10]
   + f[R2,C2,K2,10] + [ - v[C2,K2,10] * b[K2,10] ] = 0
 comm_sink_K2_11: f[0,C2,K2,11] + f[C1,C2,K2,11] - f[C2,0,K2,11]
   - f[C2,C1,K2,11] - f[C2,R1,K2,11] - f[C2,R2,K2,11] + f[R1,C2,K2,11]
   + f[R2,C2,K2,11] + [ - v[C2,K2,11] * b[K2,11] ] = 0
 comm_sink_K2_12: f[0,C2,K2,12] + f[C1,C2,K2,12] - f[C2,0,K2,12]
   - f[C2,C1,K2,12] - f[C2,R1,K2,12] - f[C2,R2,K2,12] + f[R1,C2,K2,12]
   + f[R2,C2,K2,12] + [ - v[C2,K2,12] * b[K2,12] ] = 0
 comm_sink_K2_13: f[0,C2,K2,13] + f[C1,C2,K2,13] - f[C2,0,K2,13]
   - f[C2,C1,K2,13] - f[C2,R1,K2,13] - f[C2,R2,K2,13] + f[R1,C2,K2,13]
   + f[R2,C2,K2,13] + [ - v[C2,K2,13] * b[K2,13] ] = 0
 comm_sink_K2_14: f[0,C2,K2,14] + f[C1,C2,K2,14] - f[C2,0,K2,14]
   - f[C2,C1,K2,14] - f[C2,R1,K2,14] - f[C2,R2,K2,14] + f[R1,C2,K2,14]
   + f[R2,C2,K2,14] + [ - v[C2,K2,14] * b[K2,14] ] = 0
 comm_sink_K2_15: f[0,C2,K2,15] + f[C1,C2,K2,15] - f[C2,0,K2,15]
   - f[C2,C1,K2,15] - f[C2,R1,K2,15] - f[C2,R2,K2,15] + f[R1,C2,K2,15]
   + f[R2,C2,K2,15] + [ - v[C2,K2,15] * b[K2,15] ] = 0
Bounds
Binaries
 x[0,C1,K1,1] x[0,C1,K1,2] x[0,C1,K1,3] x[0,C1,K1,4] x[0,C1,K1,5]
 x[0,C1,K1,6] x[0,C1,K1,7] x[0,C1,K1,8] x[0,C1,K1,9] x[0,C1,K1,10]
 x[0,C1,K1,11] x[0,C1,K1,12] x[0,C1,K1,13] x[0,C1,K1,14] x[0,C1,K1,15]
 x[0,C1,K2,1] x[0,C1,K2,2] x[0,C1,K2,3] x[0,C1,K2,4] x[0,C1,K2,5]
 x[0,C1,K2,6] x[0,C1,K2,7] x[0,C1,K2,8] x[0,C1,K2,9] x[0,C1,K2,10]
 x[0,C1,K2,11] x[0,C1,K2,12] x[0,C1,K2,13] x[0,C1,K2,14] x[0,C1,K2,15]
 x[0,C2,K1,1] x[0,C2,K1,2] x[0,C2,K1,3] x[0,C2,K1,4] x[0,C2,K1,5]
 x[0,C2,K1,6] x[0,C2,K1,7] x[0,C2,K1,8] x[0,C2,K1,9] x[0,C2,K1,10]
 x[0,C2,K1,11] x[0,C2,K1,12] x[0,C2,K1,13] x[0,C2,K1,14] x[0,C2,K1,15]
 x[0,C2,K2,1] x[0,C2,K2,2] x[0,C2,K2,3] x[0,C2,K2,4] x[0,C2,K2,5]
 x[0,C2,K2,6] x[0,C2,K2,7] x[0,C2,K2,8] x[0,C2,K2,9] x[0,C2,K2,10]
 x[0,C2,K2,11] x[0,C2,K2,12] x[0,C2,K2,13] x[0,C2,K2,14] x[0,C2,K2,15]
 x[0,R1,K1,1] x[0,R1,K1,2] x[0,R1,K1,3] x[0,R1,K1,4] x[0,R1,K1,5]
 x[0,R1,K1,6] x[0,R1,K1,7] x[0,R1,K1,8] x[0,R1,K1,9] x[0,R1,K1,10]
 x[0,R1,K1,11] x[0,R1,K1,12] x[0,R1,K1,13] x[0,R1,K1,14] x[0,R1,K1,15]
 x[0,R1,K2,1] x[0,R1,K2,2] x[0,R1,K2,3] x[0,R1,K2,4] x[0,R1,K2,5]
 x[0,R1,K2,6] x[0,R1,K2,7] x[0,R1,K2,8] x[0,R1,K2,9] x[0,R1,K2,10]
 x[0,R1,K2,11] x[0,R1,K2,12] x[0,R1,K2,13] x[0,R1,K2,14] x[0,R1,K2,15]
 x[0,R2,K1,1] x[0,R2,K1,2] x[0,R2,K1,3] x[0,R2,K1,4] x[0,R2,K1,5]
 x[0,R2,K1,6] x[0,R2,K1,7] x[0,R2,K1,8] x[0,R2,K1,9] x[0,R2,K1,10]
 x[0,R2,K1,11] x[0,R2,K1,12] x[0,R2,K1,13] x[0,R2,K1,14] x[0,R2,K1,15]
 x[0,R2,K2,1] x[0,R2,K2,2] x[0,R2,K2,3] x[0,R2,K2,4] x[0,R2,K2,5]
 x[0,R2,K2,6] x[0,R2,K2,7] x[0,R2,K2,8] x[0,R2,K2,9] x[0,R2,K2,10]
 x[0,R2,K2,11] x[0,R2,K2,12] x[0,R2,K2,13] x[0,R2,K2,14] x[0,R2,K2,15]
 x[C1,0,K1,1] x[C1,0,K1,2] x[C1,0,K1,3] x[C1,0,K1,4] x[C1,0,K1,5]
 x[C1,0,K1,6] x[C1,0,K1,7] x[C1,0,K1,8] x[C1,0,K1,9] x[C1,0,K1,10]
 x[C1,0,K1,11] x[C1,0,K1,12] x[C1,0,K1,13] x[C1,0,K1,14] x[C1,0,K1,15]
 x[C1,0,K2,1] x[C1,0,K2,2] x[C1,0,K2,3] x[C1,0,K2,4] x[C1,0,K2,5]
 x[C1,0,K2,6] x[C1,0,K2,7] x[C1,0,K2,8] x[C1,0,K2,9] x[C1,0,K2,10]
 x[C1,0,K2,11] x[C1,0,K2,12] x[C1,0,K2,13] x[C1,0,K2,14] x[C1,0,K2,15]
 x[C1,C2,K1,1] x[C1,C2,K1,2] x[C1,C2,K1,3] x[C1,C2,K1,4] x[C1,C2,K1,5]
 x[C1,C2,K1,6] x[C1,C2,K1,7] x[C1,C2,K1,8] x[C1,C2,K1,9] x[C1,C2,K1,10]
 x[C1,C2,K1,11] x[C1,C2,K1,12] x[C1,C2,K1,13] x[C1,C2,K1,14] x[C1,C2,K1,15]
 x[C1,C2,K2,1] x[C1,C2,K2,2] x[C1,C2,K2,3] x[C1,C2,K2,4] x[C1,C2,K2,5]
 x[C1,C2,K2,6] x[C1,C2,K2,7] x[C1,C2,K2,8] x[C1,C2,K2,9] x[C1,C2,K2,10]
 x[C1,C2,K2,11] x[C1,C2,K2,12] x[C1,C2,K2,13] x[C1,C2,K2,14] x[C1,C2,K2,15]
 x[C1,R1,K1,1] x[C1,R1,K1,2] x[C1,R1,K1,3] x[C1,R1,K1,4] x[C1,R1,K1,5]
 x[C1,R1,K1,6] x[C1,R1,K1,7] x[C1,R1,K1,8] x[C1,R1,K1,9] x[C1,R1,K1,10]
 x[C1,R1,K1,11] x[C1,R1,K1,12] x[C1,R1,K1,13] x[C1,R1,K1,14] x[C1,R1,K1,15]
 x[C1,R1,K2,1] x[C1,R1,K2,2] x[C1,R1,K2,3] x[C1,R1,K2,4] x[C1,R1,K2,5]
 x[C1,R1,K2,6] x[C1,R1,K2,7] x[C1,R1,K2,8] x[C1,R1,K2,9] x[C1,R1,K2,10]
 x[C1,R1,K2,11] x[C1,R1,K2,12] x[C1,R1,K2,13] x[C1,R1,K2,14] x[C1,R1,K2,15]
 x[C1,R2,K1,1] x[C1,R2,K1,2] x[C1,R2,K1,3] x[C1,R2,K1,4] x[C1,R2,K1,5]
 x[C1,R2,K1,6] x[C1,R2,K1,7] x[C1,R2,K1,8] x[C1,R2,K1,9] x[C1,R2,K1,10]
 x[C1,R2,K1,11] x[C1,R2,K1,12] x[C1,R2,K1,13] x[C1,R2,K1,14] x[C1,R2,K1,15]
 x[C1,R2,K2,1] x[C1,R2,K2,2] x[C1,R2,K2,3] x[C1,R2,K2,4] x[C1,R2,K2,5]
 x[C1,R2,K2,6] x[C1,R2,K2,7] x[C1,R2,K2,8] x[C1,R2,K2,9] x[C1,R2,K2,10]
 x[C1,R2,K2,11] x[C1,R2,K2,12] x[C1,R2,K2,13] x[C1,R2,K2,14] x[C1,R2,K2,15]
 x[C2,0,K1,1] x[C2,0,K1,2] x[C2,0,K1,3] x[C2,0,K1,4] x[C2,0,K1,5]
 x[C2,0,K1,6] x[C2,0,K1,7] x[C2,0,K1,8] x[C2,0,K1,9] x[C2,0,K1,10]
 x[C2,0,K1,11] x[C2,0,K1,12] x[C2,0,K1,13] x[C2,0,K1,14] x[C2,0,K1,15]
 x[C2,0,K2,1] x[C2,0,K2,2] x[C2,0,K2,3] x[C2,0,K2,4] x[C2,0,K2,5]
 x[C2,0,K2,6] x[C2,0,K2,7] x[C2,0,K2,8] x[C2,0,K2,9] x[C2,0,K2,10]
 x[C2,0,K2,11] x[C2,0,K2,12] x[C2,0,K2,13] x[C2,0,K2,14] x[C2,0,K2,15]
 x[C2,C1,K1,1] x[C2,C1,K1,2] x[C2,C1,K1,3] x[C2,C1,K1,4] x[C2,C1,K1,5]
 x[C2,C1,K1,6] x[C2,C1,K1,7] x[C2,C1,K1,8] x[C2,C1,K1,9] x[C2,C1,K1,10]
 x[C2,C1,K1,11] x[C2,C1,K1,12] x[C2,C1,K1,13] x[C2,C1,K1,14] x[C2,C1,K1,15]
 x[C2,C1,K2,1] x[C2,C1,K2,2] x[C2,C1,K2,3] x[C2,C1,K2,4] x[C2,C1,K2,5]
 x[C2,C1,K2,6] x[C2,C1,K2,7] x[C2,C1,K2,8] x[C2,C1,K2,9] x[C2,C1,K2,10]
 x[C2,C1,K2,11] x[C2,C1,K2,12] x[C2,C1,K2,13] x[C2,C1,K2,14] x[C2,C1,K2,15]
 x[C2,R1,K1,1] x[C2,R1,K1,2] x[C2,R1,K1,3] x[C2,R1,K1,4] x[C2,R1,K1,5]
 x[C2,R1,K1,6] x[C2,R1,K1,7] x[C2,R1,K1,8] x[C2,R1,K1,9] x[C2,R1,K1,10]
 x[C2,R1,K1,11] x[C2,R1,K1,12] x[C2,R1,K1,13] x[C2,R1,K1,14] x[C2,R1,K1,15]
 x[C2,R1,K2,1] x[C2,R1,K2,2] x[C2,R1,K2,3] x[C2,R1,K2,4] x[C2,R1,K2,5]
 x[C2,R1,K2,6] x[C2,R1,K2,7] x[C2,R1,K2,8] x[C2,R1,K2,9] x[C2,R1,K2,10]
 x[C2,R1,K2,11] x[C2,R1,K2,12] x[C2,R1,K2,13] x[C2,R1,K2,14] x[C2,R1,K2,15]
 x[C2,R2,K1,1] x[C2,R2,K1,2] x[C2,R2,K1,3] x[C2,R2,K1,4] x[C2,R2,K1,5]
 x[C2,R2,K1,6] x[C2,R2,K1,7] x[C2,R2,K1,8] x[C2,R2,K1,9] x[C2,R2,K1,10]
 x[C2,R2,K1,11] x[C2,R2,K1,12] x[C2,R2,K1,13] x[C2,R2,K1,14] x[C2,R2,K1,15]
 x[C2,R2,K2,1] x[C2,R2,K2,2] x[C2,R2,K2,3] x[C2,R2,K2,4] x[C2,R2,K2,5]
 x[C2,R2,K2,6] x[C2,R2,K2,7] x[C2,R2,K2,8] x[C2,R2,K2,9] x[C2,R2,K2,10]
 x[C2,R2,K2,11] x[C2,R2,K2,12] x[C2,R2,K2,13] x[C2,R2,K2,14] x[C2,R2,K2,15]
 x[R1,0,K1,1] x[R1,0,K1,2] x[R1,0,K1,3] x[R1,0,K1,4] x[R1,0,K1,5]
 x[R1,0,K1,6] x[R1,0,K1,7] x[R1,0,K1,8] x[R1,0,K1,9] x[R1,0,K1,10]
 x[R1,0,K1,11] x[R1,0,K1,12] x[R1,0,K1,13] x[R1,0,K1,14] x[R1,0,K1,15]
 x[R1,0,K2,1] x[R1,0,K2,2] x[R1,0,K2,3] x[R1,0,K2,4] x[R1,0,K2,5]
 x[R1,0,K2,6] x[R1,0,K2,7] x[R1,0,K2,8] x[R1,0,K2,9] x[R1,0,K2,10]
 x[R1,0,K2,11] x[R1,0,K2,12] x[R1,0,K2,13] x[R1,0,K2,14] x[R1,0,K2,15]
 x[R1,C1,K1,1] x[R1,C1,K1,2] x[R1,C1,K1,3] x[R1,C1,K1,4] x[R1,C1,K1,5]
 x[R1,C1,K1,6] x[R1,C1,K1,7] x[R1,C1,K1,8] x[R1,C1,K1,9] x[R1,C1,K1,10]
 x[R1,C1,K1,11] x[R1,C1,K1,12] x[R1,C1,K1,13] x[R1,C1,K1,14] x[R1,C1,K1,15]
 x[R1,C1,K2,1] x[R1,C1,K2,2] x[R1,C1,K2,3] x[R1,C1,K2,4] x[R1,C1,K2,5]
 x[R1,C1,K2,6] x[R1,C1,K2,7] x[R1,C1,K2,8] x[R1,C1,K2,9] x[R1,C1,K2,10]
 x[R1,C1,K2,11] x[R1,C1,K2,12] x[R1,C1,K2,13] x[R1,C1,K2,14] x[R1,C1,K2,15]
 x[R1,C2,K1,1] x[R1,C2,K1,2] x[R1,C2,K1,3] x[R1,C2,K1,4] x[R1,C2,K1,5]
 x[R1,C2,K1,6] x[R1,C2,K1,7] x[R1,C2,K1,8] x[R1,C2,K1,9] x[R1,C2,K1,10]
 x[R1,C2,K1,11] x[R1,C2,K1,12] x[R1,C2,K1,13] x[R1,C2,K1,14] x[R1,C2,K1,15]
 x[R1,C2,K2,1] x[R1,C2,K2,2] x[R1,C2,K2,3] x[R1,C2,K2,4] x[R1,C2,K2,5]
 x[R1,C2,K2,6] x[R1,C2,K2,7] x[R1,C2,K2,8] x[R1,C2,K2,9] x[R1,C2,K2,10]
 x[R1,C2,K2,11] x[R1,C2,K2,12] x[R1,C2,K2,13] x[R1,C2,K2,14] x[R1,C2,K2,15]
 x[R1,R2,K1,1] x[R1,R2,K1,2] x[R1,R2,K1,3] x[R1,R2,K1,4] x[R1,R2,K1,5]
 x[R1,R2,K1,6] x[R1,R2,K1,7] x[R1,R2,K1,8] x[R1,R2,K1,9] x[R1,R2,K1,10]
 x[R1,R2,K1,11] x[R1,R2,K1,12] x[R1,R2,K1,13] x[R1,R2,K1,14] x[R1,R2,K1,15]
 x[R1,R2,K2,1] x[R1,R2,K2,2] x[R1,R2,K2,3] x[R1,R2,K2,4] x[R1,R2,K2,5]
 x[R1,R2,K2,6] x[R1,R2,K2,7] x[R1,R2,K2,8] x[R1,R2,K2,9] x[R1,R2,K2,10]
 x[R1,R2,K2,11] x[R1,R2,K2,12] x[R1,R2,K2,13] x[R1,R2,K2,14] x[R1,R2,K2,15]
 x[R2,0,K1,1] x[R2,0,K1,2] x[R2,0,K1,3] x[R2,0,K1,4] x[R2,0,K1,5]
 x[R2,0,K1,6] x[R2,0,K1,7] x[R2,0,K1,8] x[R2,0,K1,9] x[R2,0,K1,10]
 x[R2,0,K1,11] x[R2,0,K1,12] x[R2,0,K1,13] x[R2,0,K1,14] x[R2,0,K1,15]
 x[R2,0,K2,1] x[R2,0,K2,2] x[R2,0,K2,3] x[R2,0,K2,4] x[R2,0,K2,5]
 x[R2,0,K2,6] x[R2,0,K2,7] x[R2,0,K2,8] x[R2,0,K2,9] x[R2,0,K2,10]
 x[R2,0,K2,11] x[R2,0,K2,12] x[R2,0,K2,13] x[R2,0,K2,14] x[R2,0,K2,15]
 x[R2,C1,K1,1] x[R2,C1,K1,2] x[R2,C1,K1,3] x[R2,C1,K1,4] x[R2,C1,K1,5]
 x[R2,C1,K1,6] x[R2,C1,K1,7] x[R2,C1,K1,8] x[R2,C1,K1,9] x[R2,C1,K1,10]
 x[R2,C1,K1,11] x[R2,C1,K1,12] x[R2,C1,K1,13] x[R2,C1,K1,14] x[R2,C1,K1,15]
 x[R2,C1,K2,1] x[R2,C1,K2,2] x[R2,C1,K2,3] x[R2,C1,K2,4] x[R2,C1,K2,5]
 x[R2,C1,K2,6] x[R2,C1,K2,7] x[R2,C1,K2,8] x[R2,C1,K2,9] x[R2,C1,K2,10]
 x[R2,C1,K2,11] x[R2,C1,K2,12] x[R2,C1,K2,13] x[R2,C1,K2,14] x[R2,C1,K2,15]
 x[R2,C2,K1,1] x[R2,C2,K1,2] x[R2,C2,K1,3] x[R2,C2,K1,4] x[R2,C2,K1,5]
 x[R2,C2,K1,6] x[R2,C2,K1,7] x[R2,C2,K1,8] x[R2,C2,K1,9] x[R2,C2,K1,10]
 x[R2,C2,K1,11] x[R2,C2,K1,12] x[R2,C2,K1,13] x[R2,C2,K1,14] x[R2,C2,K1,15]
 x[R2,C2,K2,1] x[R2,C2,K2,2] x[R2,C2,K2,3] x[R2,C2,K2,4] x[R2,C2,K2,5]
 x[R2,C2,K2,6] x[R2,C2,K2,7] x[R2,C2,K2,8] x[R2,C2,K2,9] x[R2,C2,K2,10]
 x[R2,C2,K2,11] x[R2,C2,K2,12] x[R2,C2,K2,13] x[R2,C2,K2,14] x[R2,C2,K2,15]
 x[R2,R1,K1,1] x[R2,R1,K1,2] x[R2,R1,K1,3] x[R2,R1,K1,4] x[R2,R1,K1,5]
 x[R2,R1,K1,6] x[R2,R1,K1,7] x[R2,R1,K1,8] x[R2,R1,K1,9] x[R2,R1,K1,10]
 x[R2,R1,K1,11] x[R2,R1,K1,12] x[R2,R1,K1,13] x[R2,R1,K1,14] x[R2,R1,K1,15]
 x[R2,R1,K2,1] x[R2,R1,K2,2] x[R2,R1,K2,3] x[R2,R1,K2,4] x[R2,R1,K2,5]
 x[R2,R1,K2,6] x[R2,R1,K2,7] x[R2,R1,K2,8] x[R2,R1,K2,9] x[R2,R1,K2,10]
 x[R2,R1,K2,11] x[R2,R1,K2,12] x[R2,R1,K2,13] x[R2,R1,K2,14] x[R2,R1,K2,15]
 v[0,K1,1] v[0,K1,2] v[0,K1,3] v[0,K1,4] v[0,K1,5] v[0,K1,6] v[0,K1,7]
 v[0,K1,8] v[0,K1,9] v[0,K1,10] v[0,K1,11] v[0,K1,12] v[0,K1,13] v[0,K1,14]
 v[0,K1,15] v[0,K2,1] v[0,K2,2] v[0,K2,3] v[0,K2,4] v[0,K2,5] v[0,K2,6]
 v[0,K2,7] v[0,K2,8] v[0,K2,9] v[0,K2,10] v[0,K2,11] v[0,K2,12] v[0,K2,13]
 v[0,K2,14] v[0,K2,15] v[C1,K1,1] v[C1,K1,2] v[C1,K1,3] v[C1,K1,4]
 v[C1,K1,5] v[C1,K1,6] v[C1,K1,7] v[C1,K1,8] v[C1,K1,9] v[C1,K1,10]
 v[C1,K1,11] v[C1,K1,12] v[C1,K1,13] v[C1,K1,14] v[C1,K1,15] v[C1,K2,1]
 v[C1,K2,2] v[C1,K2,3] v[C1,K2,4] v[C1,K2,5] v[C1,K2,6] v[C1,K2,7]
 v[C1,K2,8] v[C1,K2,9] v[C1,K2,10] v[C1,K2,11] v[C1,K2,12] v[C1,K2,13]
 v[C1,K2,14] v[C1,K2,15] v[C2,K1,1] v[C2,K1,2] v[C2,K1,3] v[C2,K1,4]
 v[C2,K1,5] v[C2,K1,6] v[C2,K1,7] v[C2,K1,8] v[C2,K1,9] v[C2,K1,10]
 v[C2,K1,11] v[C2,K1,12] v[C2,K1,13] v[C2,K1,14] v[C2,K1,15] v[C2,K2,1]
 v[C2,K2,2] v[C2,K2,3] v[C2,K2,4] v[C2,K2,5] v[C2,K2,6] v[C2,K2,7]
 v[C2,K2,8] v[C2,K2,9] v[C2,K2,10] v[C2,K2,11] v[C2,K2,12] v[C2,K2,13]
 v[C2,K2,14] v[C2,K2,15] v[R1,K1,1] v[R1,K1,2] v[R1,K1,3] v[R1,K1,4]
 v[R1,K1,5] v[R1,K1,6] v[R1,K1,7] v[R1,K1,8] v[R1,K1,9] v[R1,K1,10]
 v[R1,K1,11] v[R1,K1,12] v[R1,K1,13] v[R1,K1,14] v[R1,K1,15] v[R1,K2,1]
 v[R1,K2,2] v[R1,K2,3] v[R1,K2,4] v[R1,K2,5] v[R1,K2,6] v[R1,K2,7]
 v[R1,K2,8] v[R1,K2,9] v[R1,K2,10] v[R1,K2,11] v[R1,K2,12] v[R1,K2,13]
 v[R1,K2,14] v[R1,K2,15] v[R2,K1,1] v[R2,K1,2] v[R2,K1,3] v[R2,K1,4]
 v[R2,K1,5] v[R2,K1,6] v[R2,K1,7] v[R2,K1,8] v[R2,K1,9] v[R2,K1,10]
 v[R2,K1,11] v[R2,K1,12] v[R2,K1,13] v[R2,K1,14] v[R2,K1,15] v[R2,K2,1]
 v[R2,K2,2] v[R2,K2,3] v[R2,K2,4] v[R2,K2,5] v[R2,K2,6] v[R2,K2,7]
 v[R2,K2,8] v[R2,K2,9] v[R2,K2,10] v[R2,K2,11] v[R2,K2,12] v[R2,K2,13]
 v[R2,K2,14] v[R2,K2,15] y[R1,1] y[R1,2] y[R1,3] y[R1,4] y[R1,5] y[R1,6]
 y[R1,7] y[R1,8] y[R1,9] y[R1,10] y[R1,11] y[R1,12] y[R1,13] y[R1,14]
 y[R1,15] y[R2,1] y[R2,2] y[R2,3] y[R2,4] y[R2,5] y[R2,6] y[R2,7] y[R2,8]
 y[R2,9] y[R2,10] y[R2,11] y[R2,12] y[R2,13] y[R2,14] y[R2,15] z[K1,1]
 z[K1,2] z[K1,3] z[K1,4] z[K1,5] z[K1,6] z[K1,7] z[K1,8] z[K1,9] z[K1,10]
 z[K1,11] z[K1,12] z[K1,13] z[K1,14] z[K1,15] z[K2,1] z[K2,2] z[K2,3]
 z[K2,4] z[K2,5] z[K2,6] z[K2,7] z[K2,8] z[K2,9] z[K2,10] z[K2,11] z[K2,12]
 z[K2,13] z[K2,14] z[K2,15]
End
