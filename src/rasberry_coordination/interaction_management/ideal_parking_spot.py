#Created on 22 August 2022 by Roopika Ravikanna - University of Lincoln, England
#This script gives out a list of ideal parking spots

import numpy as np
import statistics
import math

#Ideal Parking Spot function
def ideal_parking_spot(header_rows, picker_dictionary):
    
    rows = list(range(1,len(header_rows)+1)) #creates as many consecutive row numbers as header rows
    n_rows = len(rows) #number of rows

    #retrieve picker dictionary here
    pickers = picker_dictionary.keys()
    n_pickers = len(pickers)#number of pickers stored
    j = n_pickers
    picker_rows_raw =[]

    for picker in pickers:  
        picker_rows_raw.append(picker_dictionary[picker])
    picker_rows_dictionary_reverse ={}

    header_rows.sort()
    picker_rows_dictionary_reverse = {i+1:header_rows[i] for i in range(len(header_rows))}
    picker_rows_dictionary = {v: k for k, v in picker_rows_dictionary_reverse.items()}
    picker_rows = []
    for picker_row in picker_rows_raw:
        picker_rows.append(picker_rows_dictionary[picker_row])

    rank = np.zeros(((n_rows), len(pickers)))#Empty list to store rank allocations to all rows by each of the pickers

    list_sum_of_rank=[] #Empty list to store rank aggregate values
    for picker in pickers: #for each of the picker repeat this

        x = pickers.index(picker) #picker indices stored in a local variable

        current_row_picker = picker_rows[n_pickers-j]
	    #calculate row-wise ranking priority of the picker
        if current_row_picker == rows[0]:

            current_pick_row = current_row_picker - 1

            rank[current_pick_row,x] = 1 #first rank given to the current row of the picker
            right_rows = n_rows - current_row_picker #number of rows to the right of the current row
            #rankings given for the rows to the right of the current row
            for i in range(1,right_rows+1):
               
                rank[current_pick_row+i,x] = i+1
               
            
        elif current_row_picker == rows[-1]:
            current_pick_row = current_row_picker - 1
            rank[current_pick_row,x] = 1 #first rank given to the current row of the picker
            left_rows = current_row_picker - 1 #number of rows to the left of the current row

            for i in range(1, left_rows+1):
           
                rank[current_pick_row-i,x] = i+1


        else:

            current_pick_row = current_row_picker - 1

            rank[current_pick_row,x] = 1 #first rank given to the current row of the picker

            right_rows = n_rows - current_row_picker #number of rows to the right of the current row
	
	        #rankings given for the rows to the right of the current row
            for i in range(1,right_rows+1):
               
                rank[current_pick_row+i,x] = i+1
               
            left_rows = current_row_picker - 1 #number of rows to the left of the current row

	        #rankings given for the rows to the left of the current row
            for i in range(1, left_rows+1):
           
                rank[current_pick_row-i,x] = i+1
    
        sum_of_rank = np.sum(rank, axis = 1)

        min_rank = min(sum_of_rank)
        sum_of_rank_2 = sum_of_rank[sum_of_rank != min_rank]
        min_rank_2 = min(sum_of_rank_2)
        sum_of_rank_3 = sum_of_rank_2[sum_of_rank_2 != min_rank_2]
        min_rank_3 = min(sum_of_rank_3)

        min_rank_list = np.where(sum_of_rank == min_rank)
        min_rank_list_2 = np.where(sum_of_rank_2 == min_rank_2)
        min_rank_list_3 = np.where(sum_of_rank_3 == min_rank_3)

        median_rank = statistics.median(min_rank_list[0])
        median_rank_2 = statistics.median(min_rank_list_2[0])
        median_rank_3 = statistics.median(min_rank_list_3[0])
        global median_row
        j = j-1
    median_row = math.floor(median_rank)+1
    median_row_2 = math.floor(median_rank_2)+1
    median_row_3 = math.floor(median_rank_3)+1

    ps1 = picker_rows_dictionary_reverse[median_row] #best spot
    ps2 = picker_rows_dictionary_reverse[median_row_2] #2nd best spot
    ps3 = picker_rows_dictionary_reverse[median_row_3] #3rd best spot
    ps = [ps1,ps2,ps3]
    return(ps)
    

#header_rows = [0.7, 1.5, 2.5, 3.5, 4.5, 5.3, 5.7, 6.5, 7.5, 8.5, 9.5, 10.3]
#picker_dictionary = {'p2':3.5,'p3':5.7, 'p1':0.7, 'p4':6.5, 'p5':9.5}


#{'Mrga': '3.5'}
#['r5.7-ca', 'r6.5-ca', 'r7.5-ca', 'r8.5-ca', 'r9.5-ca', 'r10.3-ca', 'r0.7-ca', 'r1.5-ca', 'r2.5-ca', 'r3.5-ca', 'r4.5-ca', 'r5.3-ca']


#print(ideal_parking_spot(header_rows, picker_dictionary))
    

