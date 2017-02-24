<div>
     {% assign navigation_pages = site.pages | sort: 'navigation_weight' %}
        {% for p in navigation_pages %}
          {% if p.navigation_weight %}
            {% if p.title %}
            <a class="page-link" href="{{ p.url | relative_url }}">{{ p.title | escape }}</a>
            {% endif %}
          {% endif %}
        {% endfor %}
      </div>

# Walter
Walter is a self-made, dangerous, industrial robot with 6 DOF. Right in the middle of development. Check the [Wiki](https://github.com/jochenalt/Walter/wiki)

![work in progress](https://github.com/jochenalt/Walter/blob/master/docs/videos/logo-animated.gif)


### Contents
    /cad 	    CAD designs (TurboCAD)
    /theory     mainly kinematics
    /code       code of Cortex, Trajectory Planner, and Webserver  
    /schematics schematics of Cortex and its power board
    /docs		stuff used within the [Wiki](https://github.com/jochenalt/Walter/wiki)
    /datasheets datasheets of components Walter is made of
    

<img align="left" width="400px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/IMG_20170219_125125.jpg" >

