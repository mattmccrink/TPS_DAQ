function [r1 r2 r3] = twoaxisrot(r11, r12, r21, r31, r32, ~, ~)
    r1 = atan2( r11, r12 );
    r2 = acos( r21 );
    r3 = atan2( r31, r32 );
%     if strcmpi( lim, 'zeror3')
%         for i = find(abs( r21 ) >= 1.0)
%             r1(i) = atan2( r11a(i), r12a(i) );
%             r2(i) = acos( r21(i) );
%             r3(i) = 0;
%         end
%     end
end