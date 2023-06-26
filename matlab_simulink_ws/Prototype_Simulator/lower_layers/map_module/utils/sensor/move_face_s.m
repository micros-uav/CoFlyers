function face_s_t = move_face_s(face_s,vector)
%MOVE_FACE_S
%   

face_s_t = face_s;
face_s_t(1:3:9,:) = face_s_t(1:3:9,:) + vector(1);
face_s_t(2:3:9,:) = face_s_t(2:3:9,:) + vector(2);
face_s_t(3:3:9,:) = face_s_t(3:3:9,:) + vector(3);
end

