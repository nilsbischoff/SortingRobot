basket {
    shape:ssBox, size:[.5 .5 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

wall1 (basket){
    shape: ssBox, Q:<t(.275 0 .125)>, size:[.05 .5 .3 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

wall2 (basket){
    shape: ssBox, Q:<t(-.275 0 .125)>, size:[.05 .5 .3 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

wall3 (basket){
    shape: ssBox, Q:<t(0 .275 .125)>, size:[.6 .05 .3 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}

wall4 (basket){
    shape: ssBox, Q:<t(0 -.275 .125)>, size:[.6 .05 .3 .02], color[.3 .3 .3]
    contact:1, logical:{ }
    friction:.1
}