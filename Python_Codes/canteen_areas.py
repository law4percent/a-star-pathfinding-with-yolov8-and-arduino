# middle area
Mleft_path_area = [(62, 318), (50, 492), (464, 492), (464, 318)]
mid_path_area = [(469, 318), (469, 492), (816, 492), (816, 318)]
Mright_path_area = [(821, 318), (821, 492), (1160, 492), (1125, 318)]

# bottom area
Bleft_path_area =  [(50, 497), (39, 698), (464, 698), (464, 580), (320, 580), (320, 497)]
Bmid_path_area = [(469, 497), (469, 698), (816, 698), (816, 497)]
Bright_path_area = [(821, 585), (821, 698), (1188, 698), (1160, 497), (973, 497), (973, 585)]

# top area
Tleft_path_area = [(80, 100), (62, 313), (320, 313), (320, 212), (464, 212), (464, 100)]
Tmid_path_area = [(469, 100), (469, 313), (816, 313), (816, 100)]
Tright_path_area = [(821, 100), (821, 212), (968, 212), (968, 313), (1125, 313), (1080, 100)]

tableA_pos = (347, 229) # Close Area (TL, TM, ML, M)
tableB_pos = (895, 229) # Close Area (TM, TR, M, MR)
tableC_pos = (347, 497) # Close Area (ML, M, BM, BL)
tableD_pos = (895, 497) # Close Area (M, MR, BM, BR)

pointTL_pos = (233, 175)
pointTM_pos = (636, 175)
pointTR_pos = (1050, 175)

pointML_pos = (233, 400)
pointM_pos = (636, 400)
pointMR_pos = (1050, 400)

pointBL_pos = (233, 652)
pointBM_pos = (639, 652)
pointBR_pos = (1050, 652)

TableA_closeArea = [
    {
        "area_name" : "TL",
        "pos" : pointTL_pos
    },
    {
        "area_name" : "TM",
        "pos" : pointTM_pos
    },
    {
        "area_name" : "ML",
        "pos" : pointML_pos
    },
    {
        "area_name" : "M",
        "pos" : pointM_pos
    }
]
TableB_closeArea = [
    {
        "area_name" : "TM",
        "pos" : pointTM_pos
    },
    {
        "area_name" : "TR",
        "pos" : pointTR_pos
    },
    {
        "area_name" : "M",
        "pos" : pointM_pos
    },
    {
        "area_name" : "MR",
        "pos" : pointMR_pos
    }
]
TableC_closeArea = [
    {
        "area_name" : "ML",
        "pos" : pointML_pos
    },
    {
        "area_name" : "M",
        "pos" : pointM_pos
    },
    {
        "area_name" : "BM",
        "pos" : pointBM_pos
    },
    {
        "area_name" : "BL",
        "pos" : pointBL_pos
    }
]
TableD_closeArea = [
    {
        "area_name" : "M",
        "pos" : pointM_pos
    },
    {
        "area_name" : "MR",
        "pos" : pointMR_pos
    },
    {
        "area_name" : "BM",
        "pos" : pointBM_pos
    },
    {
        "area_name" : "BR",
        "pos" : pointBR_pos
    }
]
