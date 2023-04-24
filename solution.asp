% create ground term predicates
pair(X,Y):- init(object(node,N),value(at,pair(X,Y))).
node(N):- init(object(node,N),value(at,pair(X,Y))).
robot(R):- init(object(robot,R),value(at,pair(X,Y))).
product(P):- init(object(product,P),value(on,pair(S,PQ))).
highway(N):- init(object(highway,N),value(at,pair(X,Y))).
shelf(S):- init(object(shelf,S),value(at,pair(X,Y))).
pickingStation(PS):- init(object(pickingStation,PS),value(at,pair(X,Y))).
order(O):- init(object(order,O),value(pickingStation,PS)).

% create location predicates
locNode(N,pair(X,Y)):- init(object(node,N),value(at,pair(X,Y))).
locPickingStation(PS,N):- init(object(pickingStation,PS),value(at,pair(X,Y))), init(object(node,N),value(at,pair(X,Y))).
locRobot(R,object(node,N),0):- init(object(robot,R),value(at,pair(X,Y))), locNode(N,pair(X,Y)).
locShelf(S,object(node,N),0):- init(object(shelf,S),value(at,pair(X,Y))), locNode(N,pair(X,Y)).
locProduct(P,object(shelf,S),with(quantity,PQ),0):- init(object(product,P),value(on,pair(S,PQ))).
locOrder(O,object(node,N),contains(P,PQ),0):- init(object(order,O),value(pickingStation,PS)), locPickingStation(PS,N), init(object(order,O),value(line,pair(P,PQ))).


% maintain count of entities
countCols(N):- N=#count{X:init(object(node,I),value(at,pair(X,Y)))}.
countRows(N):- N=#count{Y:init(object(node,I),value(at,pair(X,Y)))}.
countNodes(N):- N=#count{I:init(object(node,I),value(at,pair(X,Y)))}.
countShelf(N):- N=#count{I:init(object(shelf,I),value(at,pair(X,Y)))}.
countProds(N):- N=#count{I:init(object(product,I),value(on,pair(X,Y)))}.
countPickStns(N):- N=#count{I:init(object(pickingStation,I),value(at,pair(X,Y)))}.
countOrders(N):- N=#count{I:init(object(order,I),value(pickingStation,J))}.
countRbts(N):- N=#count{I:init(object(robot,I),value(at,pair(X,Y)))}.


% actions

move(0,1;0,-1;-1,0;1,0).
{moveRobot(R,move(DX,DY),T):move(DX,DY)}1:- R=1..N, countRbts(N), T=0..n-1.
{pickShelf(R,S,T):shelf(S)}1:- R=1..N, countRbts(N), T=0..n-1.
{putShelf(R,S,T):shelf(S)}1:- R=1..N, countRbts(N), T=0..n-1.
{deliver(R,O,with(S,PR,DQ),T):locOrder(O,object(node,N),contains(PR,OQ),T), locProduct(PR,object(shelf,S),with(quantity,PQ),T), DQ=1..PQ}1:- R=1..NR, countRbts(NR), T=0..n-1.

%create output format predicates
occurs(object(robot,R),move(DX,DY),T):-moveRobot(R,move(DX,DY),T).
occurs(object(robot,R),pickup,T):-pickShelf(R,_,T).
occurs(object(robot,R),putdown,T):-putShelf(R,_,T).
occurs(object(robot,R),deliver(O,P,DQ),T):-deliver(R,O,with(S,P,DQ),T).


% action constraints

% only 1 action at a time
:- occurs(object(robot,R),A1,T), occurs(object(robot,R),A2,T), A1!=A2.

% movement
% Robot constrained to move within the grid
:- locRobot(R,object(node,N),T), moveRobot(R,move(DX,DY),T), locNode(N,pair(X,Y)), X+DX<1.
:- locRobot(R,object(node,N),T), moveRobot(R,move(DX,DY),T), locNode(N,pair(X,Y)), Y+DY<1.
:- locRobot(R,object(node,N),T), moveRobot(R,move(DX,DY),T), locNode(N,pair(X,Y)), X+DX>NC, countCols(NC).
:- locRobot(R,object(node,N),T), moveRobot(R,move(DX,DY),T), locNode(N,pair(X,Y)), Y+DY>NR, countRows(NR).

% pickShelf
% Only 1 robot picks up the shelf
:- 2{pickShelf(R,S,T): robot(R)}, shelf(S).
% A robot can pick up only 1 shelf
:- pickShelf(R,S1,T), locShelf(S2,object(robot,R),T).
% A robot cannot take shelf from another robot
:- pickShelf(R1,S,T), locShelf(S,object(robot,R2),T).
% Pickup can happen at robot's location only
:- pickShelf(R,S,T), locShelf(S,object(node,N),T), not locRobot(R,object(node,N),T). 

% putShelf
% only 1 Robot can put down shelf
:- 2{putShelf(R,S,T): robot(R)}, shelf(S).
% putDown action can only happen if robot has a shelf
:- putShelf(R,S,T), not locShelf(S,object(robot,R),T).
% Shelf cannot be putdown on highway
:- putShelf(R,S,T), locRobot(R,object(node,N),T), highway(N). 


% deliver
% Delivery only allowed at delivery station
:- deliver(R,O,with(_,PR,_),T), locOrder(O,object(node,N),contains(PR,_),T), not locRobot(R,object(node, N),T).
% Delivery only allowed if robot has shelf with needed product
:- deliver(R,O,with(S,PR,_),T), locProduct(PR,object(shelf,S),with(quantity,_),T), not locShelf(S,object(robot,R),T).
% Delivery only allowed if robot has correct quantity of needed product
:- deliver(R,O,with(S,PR,DQ),T), locOrder(O,object(node,N),contains(PR,OQ),T), DQ>OQ.
:- deliver(R,O,with(S,PR,DQ),T), locProduct(PR,object(shelf,S),with(quantity,PQ),T), DQ>PQ.


% state constraints

% robot
% No robot on 2 nodes
:- 2{locRobot(R,object(node,N),T):node(N)}, robot(R), T=0..n.
% No 2 robots on the same node
:- 2{locRobot(R,object(node,N),T):robot(R)}, node(N), T=0..n.
% Robots cant swap places
:- locRobot(R1,object(node,ND1),T), locRobot(R1,object(node,ND2),T+1), locRobot(R2,object(node,ND2),T), locRobot(R2,object(node,ND1),T+1), R1!=R2.


% shelf
% No shelf on 2 robots
:- 2{locShelf(S,object(robot,NR),T): robot(NR)}, shelf(S), T=0..n.
% No 2 shelves on the same robot
:- 2{locShelf(S,object(robot,NR),T): shelf(S)}, robot(NR), T=0..n.
% No shelf on 2 nodes
:- 2{locShelf(S,object(node,N),T): node(N)}, shelf(S), T=0..n.
% No 2 shelves on the same node
:- 2{locShelf(S,object(node,N),T): shelf(S)}, node(N), T=0..n.
% No shelf on 2 locations (robot, node)
:- locShelf(S,object(node,_),T), locShelf(S,object(robot,_),T).

% highway
% Picking Station cannot be a highway
:- locPickingStation(_,N), highway(N).
% Shelf cannot be on a highway.
:- locShelf(S,object(node,N),_), highway(N).


% action effects

% moveRobot effect
locRobot(R,object(node,NEW_ND),T+1):- locRobot(R,object(node,N),T), locNode(N,pair(X,Y)), locNode(NEW_ND, pair(X+DX,Y+DY)), moveRobot(R,move(DX,DY),T).

% pickShelf effect
locShelf(S,object(robot,R),T+1):- pickShelf(R,S,T), locShelf(S,object(node,N),T), locRobot(R,object(node,N),T).

% putShelf effect
locShelf(S,object(node,N),T+1):- putShelf(R,S,T), locShelf(S,object(robot,R),T), locRobot(R,object(node,N),T).

% deliver effect
locOrder(O,object(node,N),contains(PR,OU-DQ),T+1):- deliver(R,O,with(S,PR,DQ),T), locOrder(O,object(node,N),contains(PR,OU),T).
locProduct(PR,object(shelf,S),with(quantity,PQ-DQ),T+1):- deliver(R,O,with(S,PR,DQ),T), locProduct(PR,object(shelf,S),with(quantity,PQ),T).


% inertial laws

locRobot(R,object(node,N),T+1):- locRobot(R,object(node,N),T), not moveRobot(R,move(_,_),T), T<n.
locShelf(S,object(node,N),T+1):- locShelf(S,object(node,N),T), not pickShelf(_,S,T), T<n.
locShelf(S,object(robot,R),T+1):- locShelf(S,object(robot,R),T), not putShelf(R,S,T), T<n.
locOrder(O,object(node,N),contains(PR,OU),T+1):- locOrder(O,object(node,N),contains(PR,OU),T), locProduct(PR,object(shelf,S),with(quantity,PQ),T), not deliver(_,O,with(S,PR,_),T), T<n.
locProduct(PR,object(shelf,S),with(quantity,PQ),T+1):- locProduct(PR,object(shelf,S),with(quantity,PQ),T), not deliver(_,_,with(S,PR,_),T), T<n.

% final state
:- not locOrder(O,object(node,_),contains(PR,0),n), locOrder(O,object(node,_),contains(PR,_),0).

timeTaken(N-1):-N=#count{T:occurs(O,A,T)}.
#minimize{1,O,A,T:occurs(O,A,T)}.
#minimize{T:occurs(O,A,T)}.

#show occurs/3.
#show timeTaken/1.