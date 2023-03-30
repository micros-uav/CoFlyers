function R = euler2RotationMatrix(eul,seq)
%EULER2ROTATIONMATRIX 此处显示有关此函数的摘要
%   此处显示详细说明
R = zeros(3,3,size(eul,1),'like',eul);
ct = cos(eul);
st = sin(eul);

% The parsed sequence will be in all upper-case letters and validated
switch seq
    case 'ZYX'
        %     The rotation matrix R can be constructed as follows by
        %     ct = [cz cy cx] and st = [sy sy sx]
        %
        %     R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
        %            cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
        %              -sy            cy*sx             cy*cx]
        %       = Rz(tz) * Ry(ty) * Rx(tx)
        
        R(1,1,:) = ct(:,2).*ct(:,1);
        R(1,2,:) = st(:,3).*st(:,2).*ct(:,1) - ct(:,3).*st(:,1);
        R(1,3,:) = ct(:,3).*st(:,2).*ct(:,1) + st(:,3).*st(:,1);
        R(2,1,:) = ct(:,2).*st(:,1);
        R(2,2,:) = st(:,3).*st(:,2).*st(:,1) + ct(:,3).*ct(:,1);
        R(2,3,:) = ct(:,3).*st(:,2).*st(:,1) - st(:,3).*ct(:,1);
        R(3,1,:) = -st(:,2);
        R(3,2,:) = st(:,3).*ct(:,2);
        R(3,3,:) = ct(:,3).*ct(:,2);
        
    case 'ZYZ'
        %     The rotation matrix R can be constructed as follows by
        %     ct = [cz cy cz2] and st = [sz sy sz2]
        %
        %     R = [  cz2*cy*cz-sz2*sz   -sz2*cy*cz-cz2*sz    sy*cz
        %            cz2*cy*sz+sz2*cz   -sz2*cy*sz+cz2*cz    sy*sz
        %                     -cz2*sy              sz2*sy       cy]
        %       = Rz(tz) * Ry(ty) * Rz(tz2)
        
        R(1,1,:) = ct(:,1).*ct(:,3).*ct(:,2) - st(:,1).*st(:,3);
        R(1,2,:) = -ct(:,1).*ct(:,2).*st(:,3) - st(:,1).*ct(:,3);
        R(1,3,:) = ct(:,1).*st(:,2);
        R(2,1,:) = st(:,1).*ct(:,3).*ct(:,2) + ct(:,1).*st(:,3);
        R(2,2,:) = -st(:,1).*ct(:,2).*st(:,3) + ct(:,1).*ct(:,3);
        R(2,3,:) = st(:,1).*st(:,2);
        R(3,1,:) = -st(:,2).*ct(:,3);
        R(3,2,:) = st(:,2).*st(:,3);
        R(3,3,:) = ct(:,2);
        
    case 'XYZ'
        %     The rotation matrix R can be constructed as follows by
        %     ct = [cx cy cz] and st = [sx sy sz]
        %
        %     R = [            cy*cz,           -cy*sz,     sy]
        %         [ cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx]
        %         [ sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]
        %       = Rx(tx) * Ry(ty) * Rz(tz)
        
        R(1,1,:) = ct(:,2).*ct(:,3);
        R(1,2,:) = -ct(:,2).*st(:,3);
        R(1,3,:) = st(:,2);
        R(2,1,:) = ct(:,1).*st(:,3) + ct(:,3).*st(:,1).*st(:,2);
        R(2,2,:) = ct(:,1).*ct(:,3) - st(:,1).*st(:,2).*st(:,3);
        R(2,3,:) = -ct(:,2).*st(:,1);
        R(3,1,:) = st(:,1).*st(:,3) - ct(:,1).*ct(:,3).*st(:,2);
        R(3,2,:) = ct(:,3).*st(:,1) + ct(:,1).*st(:,2).*st(:,3);
        R(3,3,:) = ct(:,1).*ct(:,2);
    case 'ZXY'
        %     The rotation matrix R can be constructed as follows by
        %     ct = [cx cy cz] and st = [sx sy sz]
        %
        %     R = [cy*cz - sx*sy*sz, -cx*sz, cz*sy + cy*sx*sz]
        %         [cy*sz + cz*sx*sy,  cx*cz, sy*sz - cy*cz*sx]
        %         [          -cx*sy,     sx,            cx*cy]
        %       = Rz(tz) * Rx(tx) * Ry(ty)
        
        R(1,1,:) = ct(:,2).*ct(:,3) - st(:,1).*st(:,2).*st(:,3);
        R(1,2,:) = -ct(:,1).*st(:,3);
        R(1,3,:) = ct(:,3).*st(:,2) + ct(:,2).*st(:,1).*st(:,3);
        R(2,1,:) = ct(:,2).*st(:,3) + ct(:,3).*st(:,1).*st(:,2);
        R(2,2,:) = ct(:,1).*ct(:,3);
        R(2,3,:) = st(:,2).*st(:,3) - ct(:,2).*ct(:,3).*st(:,1);
        R(3,1,:) = -ct(:,1).*st(:,2);
        R(3,2,:) = st(:,1);
        R(3,3,:) = ct(:,1).*ct(:,2);
end
end

