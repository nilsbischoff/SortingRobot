world {}

## table

Include: 'small_table.g'
Edit table (world) { Q:<t(0 .5 .05)> }

## baskets

Prefix: "red_"
Include: 'basket.g'

Prefix: "green_"
Include: 'basket.g'

Prefix: "blue_"
Include: 'basket.g'

Prefix!

Edit red_basket (world) { Q:<t(0 -.8 .025)>, color:[1 0 0] }
Edit green_basket (world) { Q:<t(.8 -.8 .025)>, color:[0 1 0] }
Edit blue_basket (world) { Q:<t(-.8 -.8 .025)>, color:[0 0 1] }

## robot

Prefix: "L_"
Include: 'panda_moveGripper.g'

Prefix: "R_"
Include: 'panda_moveGripper.g'

Prefix!

Edit L_panda_link0 (world) { Q:<t(-.4 0 0) d(90 0 0 1)> }
Edit R_panda_link0 (world)  { Q:<t( .4 0 0) d(90 0 0 1)> }