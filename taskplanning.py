# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 25 16:13:16 2021

@author: elliottmcg
=============== Description ========================
Takes user input on parts in bin, parts in kit, and
parts needed in kit. Then moves parts from bin to
kit to reach the described goal
====================================================
"""
import sys

# robot tree for location and arm condition
robot = {'loc': 'at_home', 'left': True, 'right': True}


def pick(kit, end, bin_, part_name):
    """Gets kit, goal, bin, and robot status and updates bin

    Args:
        kit (int): Number of parts in the kit
        end (int): Number of parts that need to be in kit
        bin_ (int): Number of parts in bin
        part_name (str): Name of part operating on

    Returns:
        bin_: Updated count of parts in the bin
    """

    if robot['loc'] == 'at_bin' and robot['left'] is True and robot['right'] is True and kit < end and bin_ > 0:
        if (end - kit) == 1:  # If only one part is needed use only the left arm, else use both
            robot['left'] = False
            bin_ -= 1  # Remove one part from bin
            print(f"Pick {part_name} with left arm")
            return bin_
        else:
            robot['left'] = False
            robot['right'] = False
            print(f"Pick {part_name} with left arm")
            print(f"Pick {part_name} with right arm")
            bin_ -= 2  # Remove 2 parts from bin
            return bin_


def place(kit, end, part_name):
    """Gets kit, goal, and robot status and updates kit

    Args:
        kit (int): Number of parts in the kit
        end (int): Number of parts that need to be in kit
        part_name (str): Name of part operating on

    Returns:
        kit: Updated count of parts in the kit
    """

    if robot['loc'] == 'at_kit' and robot['left'] is False and robot['right'] is False and kit < end:
        robot['left'] = True
        robot['right'] = True
        kit += 2  # Add 2 parts to kit
        print(f"Place {part_name} with left arm")
        print(f"Place {part_name} with right arm")
        return kit
    elif robot['loc'] == 'at_kit' and robot['left'] is False and robot['right'] is True and kit < end:
        robot['left'] = True
        kit += 1  # Add 1 part to kit
        print(f"Place {part_name} with left arm")
        return kit


def move_to_bin(kit, end):
    """Gets kit, goal and robot status and updates robot location

    Args:
        kit (int): Number of parts in the kit
        end (int): Number of parts that need to be in kit

    Returns:
        None
    """

    if robot['loc'] != 'at_bin' and robot['left'] is True and robot['right'] is True and kit < end:
        robot['loc'] = 'at_bin'
        print('Move to bin')
        return
    else:
        return


def move_to_tray(kit, end):
    """Gets kit, goal and robot status and updates robot location

    Args:
        kit (int): Number of parts in the kit
        end (int): Number of parts that need to be in kit

    Returns:
        None
    """

    if robot['loc'] != 'at_kit' and robot['left'] is False and robot['right'] is False and kit < end:
        robot['loc'] = 'at_kit'
        print('Move to tray')
        return
    elif robot['loc'] != 'at_kit' and robot['left'] is False and robot['right'] is True and kit < end:
        robot['loc'] = 'at_kit'
        print('Move to tray')
        return
    else:
        return


def solve():
    """Main function that calls all other functions to solve user input problem

    Returns:
        None
    """

    r_bin, g_bin, b_bin = input('How many red/green/blue parts in the bins? ').split()  # Get user bin input
    # Convert user input to int
    rb_num = int(r_bin)
    gb_num = int(g_bin)
    bb_num = int(b_bin)

    if rb_num > 10:
        print("\nERROR: The maximum allowed number of parts in a bin is 10")
        r_bin = input('How many red parts in the bin? ')
        rb_num = int(r_bin)
    if gb_num > 10:
        print("\nERROR: The maximum allowed number of parts in a bin is 10")
        g_bin = input('How many green parts in the bin? ')
        gb_num = int(g_bin)
    if bb_num > 10:
        print("\nERROR: The maximum allowed number of parts in a bin is 10")
        b_bin = input('How many blue parts in the bin? ')
        bb_num = int(b_bin)

    r_kit, g_kit, b_kit = input('How many red/green/blue parts already in the kit tray? ').split()  # Get user kit input
    # Get user goal input
    r_goal, g_goal, b_goal = input('How many red/green/blue parts to place in the kit tray? ').split()
    # Convert user input to int
    rk_num = int(r_kit)
    gk_num = int(g_kit)
    bk_num = int(b_kit)
    # Convert user input to int
    rg_num = int(r_goal)
    gg_num = int(g_goal)
    bg_num = int(b_goal)
    # Check if parts in kit are greater is greater than goal
    if rk_num > rg_num or gk_num > gg_num or bk_num > bg_num:
        print('Kit tray has more parts than needed...exiting')
        sys.exit()
    # Check if the goal is possible with the input bin and kit parts
    if rg_num > (rb_num + rk_num):
        needed = rg_num - rk_num
        print(f"\nNot enough red parts for kitting: {needed} needed, {rb_num} available")
        sys.exit()
    if gg_num > (gb_num + gk_num):
        needed = gg_num - gk_num
        print(f"\nNot enough green parts for kitting: {needed} needed, {gb_num} available")
        sys.exit()
    if bg_num > (bb_num + bk_num):
        needed = bg_num - bk_num
        print(f"\nNot enough blue parts for kitting: {needed} needed, {bb_num} available")
        sys.exit()

    print('Generating Plan...')
    # Loop to move robot and pick/place parts until goal is reached
    while True:
        # Check if goal is reached
        if rk_num == rg_num and gk_num == gg_num and bk_num == bg_num:
            print('Task Complete')
            print(f"The kit tray has {rk_num} red part(s) -- the bin has {rb_num} red part(s) left")
            print(f"The kit tray has {gk_num} green part(s) -- the bin has {gb_num} green part(s) left")
            print(f"The kit tray has {bk_num} blue part(s) -- the bin has {bb_num} blue part(s) left")
            break
        # Check if goal for red parts has been reached
        if rk_num != rg_num:
            print('==========')
            move_to_bin(rk_num, rg_num)  # Call move_to_bin function
            rb_num = pick(rk_num, rg_num, rb_num, 'red')  # Call pick function
            move_to_tray(rk_num, rg_num)  # Call move_to_tray function
            rk_num = place(rk_num, rg_num, 'red')  # Call place function
            print('==========')
        # Check if goal for green parts has been reached
        if gk_num != gg_num:
            print('==========')
            move_to_bin(gk_num, gg_num)
            gb_num = pick(gk_num, gg_num, gb_num, 'green')
            move_to_tray(gk_num, gg_num)
            gk_num = place(gk_num, gg_num, 'green')
            print('==========')
        # Check if goal for blue parts has been reached
        if bk_num != bg_num:
            print('==========')
            move_to_bin(bk_num, bg_num)
            bb_num = pick(bk_num, bg_num, bb_num, 'blue')
            move_to_tray(bk_num, bg_num)
            bk_num = place(bk_num, bg_num, 'blue')
            print('==========')
