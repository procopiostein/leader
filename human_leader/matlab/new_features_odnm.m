%decompose relative velocity in x and y, compute lateral displacement

function [out]=new_features(in)

out = in;
out(:,10) = cos(in(:,6)).*in(:,7); %new feature: sagital dist