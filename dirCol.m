function [soln] = dirCol(prb)
%DIRCOL unsurprisingly performs direct collocation
%essentially all setup to solve using fmincon
%create initial guess for time, state, control

%construct guess vector
tGuess = prb.guess.time;
time = linsapce(tGuess(1), tGuess(2), prb.nPts);
xGuess = interp1(tGuess',prb.guess.state',time')';
uGuess = interp1(tGuess',prb.guess.control',time')';
[qGuess,ind] = conCat(tGuess,xGuess,uGuess);

%construct lower bound vector
tLb = linspace(prb.bnd.tiLow, prb.bnd.tfLow, prb.nPnts);
xLb = prb.bnd.xLow*ones(1,prb.nPts);
xLb(:,1) = prb.bnd.xiLow; xLb(:,end) = prb.bnd.xfLow;
uLb = prb.bnd.uLow*ones(1,prb.nPts);
[qLb,~] = conCat(tLb,xLb,uLb);

%construct upper bound vector
tUb = linspace(prb.bnd.tiUpp, prb.bnd.tfUpp, prb.nPnts);
xUb = prb.bnd.xUpp*ones(1,prb.nPts);
xUb(:,1) = prb.bnd.xiUpp; xUb(:,end) = prb.bnd.xfUpp;
uUb = prb.bnd.uUpp*ones(1,prb.nPts);
[qUb,~] = conCat(tUb,xUb,uUb);

fPrb.x0 = qGuess;
fPrb.lb = qLb;
fPrb.ub = qUb;
fPrb.Aineq = [];
fPrb.bineq = [];
fPrb.Aeq = [];
fPrb.beq = [];
fPrb.solver = 'fmincon';
fPrb.options = prb.fminconOpts;

fPrb.objective =...         %define objective to be passed to fmincon
@(t,x,u)(objectiveFunc...   %use our objective func with the inputs:
(q,ind, prb.obj.path, prb.obj.boundary, prb.obj.weights));

fPrb.nonlcon =...
@(t,x,u)(nonLinConstraint...
(q,ind, prb.cnstr.path, prb.cnstr.bound, prb.cnstr.dyn, prb.cnstr.dynErr));

[q,objVal,flag,output] = fmincon(fPrb);

[soln.t,soln.x,soln.u] = unCat(q,ind);
soln.info.objVal = objVal;
soln.info.flag = flag;
soln.info.output = output;

end


function cost = objectiveFunc(q,ind,path,boundary,weights)
%where the cost is actually calculated
[t,x,u] = unCat(q,ind);

%find the cost associated witht the path objective
if isempty(path)
    pathCost = 0;
else
    hk = (t(end)-t(1))/(length(t)-1); %hk = t(k+1)-t(k) for all k
    integ = path(t,x,u);
    pathCost = hk*dot(weights,integ);
end

%find the cost assocaited with the boundary objective
if isempty(boundary)
    bndCost = 0;
else
    bndCost = boundary(t(1), x(:,1), t(end), x(:,end));
end

%sum costs
cost = pathCost + bndCost;
end




function [c,ceq] = nonLinConstraint(q,ind, path, bound, dyn, dynErr)
%put the constraints together 
[t,x,u] = unCat(q,ind);
%need f(dynamics) at all t to enforce dynamics equality
%dynError function takes hk x and f
f = dyn(t,x,u);
hk = (t(end)-t(1))/(length(t)-1);
dynEq = dynErr(hk,x,f);

%find path constraints
if isempty(path)
    pathIneq = [];
    pathEq = [];
else 
    [pathIneq, pathEq] = path(t,x,u);
end

%find boundary constraints
if isempty(bound)
    boundIneq = [];
    boundEq = [];
else 
    [boundIneq, boundEq] = bound(t(1), x(:,1), t(end), x(:,end));
end

c = [pathIneq; boundIneq];
ceq = [pathEq; boundEq; dynEq];
end


%makes a standard form for the time, state, and control to be passed
function [q,ind] = conCat(t,x,u)

q = t';
ind.t = 1:length(t); % so 1,2

m = size(x,1);
for i = 1:m     %for each state, put them end to end in the vector q
q = cat(1,q,x(i,:)');
end
%index info of x 
%starts at position after t
%ends at the poistion where t ends plus the number of elements in x
%has m states to be reconstructed 
ind.x = [ind.t(2)+1 ind.t(2)+numel(x), m];

p = size(u,1);
for i = 1:p     %for each control, put them end to end in the vector q
q = cat(1,q,u(i,:)');
end
%index info of u
%starts at position after x
%ends at the poistion where x ends plus the number of elements in u
%has p controls to be reconstructed 
ind.u = [ind.x(2)+1 ind.x(2)+numel(u), p];

end

function [t,x,u] = unCat(q,ind)

t = q(ind.t(1):ind.t(2))';

xCat = q(ind.x(1):ind.x(2)); %concatenated x vector
m = ind.x(3); %number of states
n = length(xCat)/m;
for i = 1:m
    x(i,:) = xCat((n*(i-1)+1):i*n);
end

uCat = q(ind.u(1):ind.u(2)); %concatenated u vector
p = ind.u(3); %number of controls
n = length(uCat)/p;
for i = 1:p
    u(i,:) = uCat((n*(i-1)+1):i*n);
end

end 
