table {
    shape:ssBox, size:[2. .5 .1 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

barrier1 (table){
    shape: ssBox, Q:<t(1.025 0 .025)>, size:[.05 .5 .15 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

barrier2 (table){
    shape: ssBox, Q:<t(-1.025 0 .025)>, size:[.05 .5 .15 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

barrier3 (table){
    shape: ssBox, Q:<t(0 .275 .025)>, size:[2.1 .05 .15 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

barrier4 (table){
    shape: ssBox, Q:<t(0 -.275 .025)>, size:[2.1 .05 .15 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}