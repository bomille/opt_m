function [prb] = setDefaults(prb)
%SETDEFAULTS constructs default options for prb

if isempty(prb.acc)
    prb.acc = 'low';
end

switch prb.acc
    
    case 'low'
        prb.nPnts = 12;
        prb.fminconOpts = optimset('Display','iter','TolFun',1e-4,...
                'MaxIter',200,'MaxFunEvals',1e4*(prb.m+prb.p));
        
    case 'med'
        prb.nPnts = 25;
        prb.fminconOpts = optimset('Display','iter','TolFun',1e-6,...
                'MaxIter',400,'MaxFunEvals',5e4*(prb.m+prb.p));
        
    case 'high'
        prb.nPnts = 50;
        prb.fminconOpts = optimset('Display','iter','TolFun',1e-8,...
                'MaxIter',800,'MaxFunEvals',1e5*(prb.m+prb.p));
        

end

