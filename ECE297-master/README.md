# Milestone 1
This is the first milestone of our ECE297 project. This milestone are supposed to:

1. Query the libstreetsdatabase API using functions in the provided header
files.
2. Create a new API of functions that will be useful in your project.
3. Use STL data structures such as vectors and maps, and choose appropriate
data structures to speed up an API.
4. Use unit tests to test your code.

To test the project, use the following command:

`ece297exercise 1 --list_testers`

`ece297exercise 1 --run_tester <tester name>`

For submission, use this command: `ece297submit 1`


# Milestone 2
• Your program must be able to load and visualize any map without recompilation (i.e. you
must get the map name from user input in some way).

• You must be able to visualize streets, points of interest and features (lakes, buildings,
etc.) – essentially all the items provided by the layer 2 StreetsDatabaseAPI.h. Use the
equirectangular map projection described in milestone 1.

• You must be able to show street names and point of interest names.
• The user must be able to tell major roads from minor roads, and tell which roads are
one-way and in which direction.

• The user must be able to click on an intersection; the intersection should be highlighted
and information about it should be either displayed in the graphics or printed to the
command (terminal) window.

• The user must be able to enter the names of two streets (for example by clicking on
Find button and entering the street names) and have all intersections between those street
names be highlighted in the graphics and information about the intersections be shown in
the graphics or printed to the terminal window.

• The find feature above should also work with partial street names in some way. For
example if the text typed uniquely identifies the street, a partial street name is sufficient.

• The map should be fast and interactive to use, and while it displays all the information
above it should do so in ways that do not overwhelm the user with unnecessary detail and
clutter.

# Milestone 3
## Algorithm
• The route must be legal – that is, it must be a sequence of street segments which form a
connected path from the start intersection to the end intersection. You must also respect
one way streets, and not try to travel down them in the wrong direction. Note that it is
also possible that no legal route exists between two intersections, in which case you should
return an empty (size = 0) vector of street segments.

• Your route should have the minimize the travel time between the two intersections, where
travel time is defined below.

• You should find the route quickly; you will fail performance tests if your path-finding
code is not fast enough.
The travel time required to traverse a sequence of street segments is the sum of two
components.

• The time to drive along each street segment, which is simply the length of the street
segment divided by its speed limit.

• The time to make turns between street segments. We assume that no turn is required
when you travel from one street segment to another if they are both part of the same street
– that is, we are assuming you hit only green lights when you are going straight down a
street. When you travel from a street segment that is part of one street (e.g. Yonge Street)
to a street segment that is part of another street (e.g. Bloor Street) however, you will have
to make a turn and this will cost turn_penalty extra time. turn_penalty models the
average time it takes for you to wait for a break in traffic and/or a traffic light to turn
green so you can turn.


## UI
• Your program must work with any map without recompilation, so the user should be
able to specify the map of interest.

• Users must be able to enter data/commands that exercise your code to find a path
between two intersections (specified by entering street names at each intersection, e.g.
Yonge Street and Bloor Street).

• Your interface must have some method that allows users to specify partial street names
(e.g. Yong) instead of full street names to identify the street.

• Users must also be able to find paths between intersections by mouse clicking on intersections.

• You must display the found path in the user interface; for full marks this path should be
easy to understand and give a good sense of how to follow it.

• You must print out detailed travel directions. For full marks, these travel directions must
be sufficiently detailed and clear that a typical driver could follow them.

• You must give informative error messages if the user mistypes something (e.g. an invalid
street or map name).

• You must have some method (e.g. a help GUI button or notes in dialog boxes) so new
users can learn the interface.
