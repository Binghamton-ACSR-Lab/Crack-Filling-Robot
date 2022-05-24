%% Modified Chinese Postman Problem Fuction
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

%Chinese Postman Problem Function
%--------------------------------------------------------------------%
function [Path,Weight,add,st] = ChinesePostman(ADJ,Matrix_Input, DIST, Start, ee, cl)
% ChinesePostman(ADJ,Matrix_Input, DIST, Start, ee, cl)
%
% Function to compute a Chinese Postman Problem using Linear Programming Matching.  
%
% INPUTS:
%   ADJ = 
%   Matrix_Input = 
%   DIST = 
%   Start = 
%   ee = 
%   cl = 
%
% OUTPUTS:
%   Path = 
%   Weight = 
%   add = 
%   st = 
%
% Defaults:

switch nargin
    case 3
        [A,t] = MatGen(Matrix_Input);ee=[];cl=[];Start=[];
    case 4
        [A,t] = MatGen(Matrix_Input);ee=[];cl=[];
    case 5
        [A,t] = MatGen(Matrix_Input);cl=[];
    case 6
        [A,t] = MatGen(Matrix_Input);
    otherwise
        [A,t] = MatGen();
end

flag=false;

switch t
    case 1
        %Eulerian
        if ~isempty(cl)
            Start=cl;
        else
            prompt = 'Specify Start Vertex: ';
            if isempty(prompt)
                prompt = 1;                   	%Defaults to start vertex 1
            end
            Start = input(prompt);
        end    
        [Path,Weight] = Fleury(A,ADJ,Start);add=[];st=[];
%     case 2
%         %Semi-Eulerian
%         %[Added_Edges,Added_Weight,st] = OptPair(A,DIST);
%         [Added_Edges,Added_Weight,st] = optOddPair(ADJ,DIST,ee,cl,flag);        % Vishnu: My funtion to use LP
% %         if isempty(Start)
% %             Start=st(1);
% %         end
%         tempG=graph(A);
%         if ~isempty(Added_Edges);add=Added_Edges;else;add=[];end
%         for i=1:size(Added_Edges,1)
%             if A(Added_Edges(i,1),Added_Edges(i,2))==0
%             A(Added_Edges(i,1),Added_Edges(i,2))=DIST(Added_Edges(i,1),Added_Edges(i,2));
%             A(Added_Edges(i,2),Added_Edges(i,1))=DIST(Added_Edges(i,1),Added_Edges(i,2));
%             end
%             ADJ(Added_Edges(i,1),Added_Edges(i,2))=ADJ(Added_Edges(i,1),Added_Edges(i,2))+1;
%             ADJ(Added_Edges(i,2),Added_Edges(i,1))=ADJ(Added_Edges(i,2),Added_Edges(i,1))+1;
%             Added_Edges(i,:)=[0 0];
%         end
%         Added_Edges(any(Added_Edges==0,2),:)=[];  
%         
%         if ~isempty(cl)
%             Start=cl;
%             G = graph(A);
%             n=neighbors(G,cl);
%             tt=findedge(tempG,cl,n);tt=tt>0;
%             if sum(tt)==0
%                 [~,idx]=max(G.Edges{findedge(G,[cl,cl],[n(1),n(2)]),'Weight'});st=[G.Edges{findedge(G,cl,n(idx)),'EndNodes'}];
%                 A(cl,n(idx))=0;A(n(idx),cl)=0;ADJ(cl,n(idx))=ADJ(cl,n(idx))-1;ADJ(n(idx),cl)=ADJ(n(idx),cl)-1;
%             elseif sum(tt)==1
%                 st=[G.Edges{findedge(G,cl,n(~tt)),'EndNodes'}];
%                 A(cl,n(~tt))=0;A(n(~tt),cl)=0;ADJ(cl,n(~tt))=ADJ(cl,n(~tt))-1;ADJ(n(~tt),cl)=ADJ(n(~tt),cl)-1;
%             end
%         end
%         [Path,Weight] = Fleury(A,ADJ,Start,Added_Edges,Added_Weight);
%         
%     case 3
    otherwise
        %Non-Eulerian
        %[Added_Edges,Added_Weight,st] = OptPair(A,DIST); 
        rmflag=true;   % Remove crack edges from LP 
        [Added_Edges,Added_Weight,st] = optOddPair(ADJ,DIST,ee,cl,flag,rmflag);        % Vishnu: My funtion to use LP
        
        if isempty(Added_Edges)
            rmflag=false;  % Dont remove crack edges from LP 
            [Added_Edges,Added_Weight,st] = optOddPair(ADJ,DIST,ee,cl,flag,rmflag); 
        end
        
        tempG=graph(A); add=[];
        
        %%% Circuit Check
        EdgeTable=table(Added_Edges,diag(DIST(Added_Edges(:,1),Added_Edges(:,2))),'VariableNames',{'EndNodes','Weight'});
        G = graph(A);G_c=addedge(G,EdgeTable);
        C = conncomp(G_c, 'OutputForm', 'cell');
        if size(C,2)>1
            %new edit changed G_c to G
            combEdge = combnk(unique(G.Edges.EndNodes),2);                             % Create all combinations of edges b/w odd nodes
            
% %             combEdge = combnk(find(1*(mod(sum(A>0,2),2)~=0)),2);
            
            combEdge(ismember(combEdge,G.Edges.EndNodes,'rows'),:)=[];  % Removing edges from crackEdge


% %             %%%% Org
% %             c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));
            
% %             %%%%Christofides
% %             nu=1;u_x=-nu*(degree(G,combEdge(:,1))-2); u_y=-nu*(degree(G,combEdge(:,2))-2);
% %             c_e=diag(DIST(combEdge(:,1),combEdge(:,2)))-u_x-u_y;

            %%%%Modified Christofides
            c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));E_lambda0=[];
            for lambda=[0]%,90
            if lambda>0;c_e(ismember(combEdge,E_lambda0,'rows'))=c_e(ismember(combEdge,E_lambda0,'rows'))+lambda;end
        
            EdgeTable=table(combEdge,c_e,'VariableNames',{'EndNodes','Weight'});
            test = G.Edges.EndNodes;Gg=graph(test(:,1),test(:,2),zeros(height(G.Edges),1));Gg.Edges;
            G_a=addedge(Gg,EdgeTable);G_a=simplify(G_a);
                  
            T = minspantree(G_a);
            T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
            Added_Edges= T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
            
            if lambda==0; E_lambda0=Added_Edges;end
            end
            %%%%
            
% %             %%%%Modified Christofides - New Cost Function
% %             c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));E_lambda0=[];[~,Index]=sort(c_e,'ascend');lambda=0;
% %             for cc=1:2
% %                 
% %                 if lambda>0;c_e(ismember(combEdge,E_lambda0,'rows'))=c_e(ismember(combEdge,E_lambda0,'rows'))+c_e(Index(indd))-c_e(Index(indd-1));end
% % 
% %                 EdgeTable=table(combEdge,c_e,'VariableNames',{'EndNodes','Weight'});
% %                 test = G.Edges.EndNodes;Gg=graph(test(:,1),test(:,2),zeros(height(G.Edges),1));Gg.Edges;
% %                 G_a=addedge(Gg,EdgeTable);G_a=simplify(G_a);
% % 
% %                 T = minspantree(G_a);
% %                 T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
% %                 Added_Edges= T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
% % 
% %                 if lambda==0 
% %                     E_lambda0=Added_Edges;
% %                     indd=find(ismember(combEdge,E_lambda0,'rows'));
% %                     indd=find(ismember(Index,indd));
% %                     lambda=1;
% %                 end
% %             end
% %             %%%%
            
% %             EdgeTable=table(combEdge,c_e,'VariableNames',{'EndNodes','Weight'});
% %             test = G.Edges.EndNodes;Gg=graph(test(:,1),test(:,2),zeros(39,1));Gg.Edges;
% %             G_a=addedge(Gg,EdgeTable);G_a=simplify(G_a);
% %                   
% %             T = minspantree(G_a);
% %             T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
% %             Added_Edges= T.Edges.EndNodes(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:);
                        
            if ~isempty(Added_Edges);add=Added_Edges;else;add=[];end
            for i=1:size(Added_Edges,1)
                if A(Added_Edges(i,1),Added_Edges(i,2))==0
                A(Added_Edges(i,1),Added_Edges(i,2))=DIST(Added_Edges(i,1),Added_Edges(i,2));
                A(Added_Edges(i,2),Added_Edges(i,1))=DIST(Added_Edges(i,1),Added_Edges(i,2));
                end
                ADJ(Added_Edges(i,1),Added_Edges(i,2))=ADJ(Added_Edges(i,1),Added_Edges(i,2))+1;
                ADJ(Added_Edges(i,2),Added_Edges(i,1))=ADJ(Added_Edges(i,2),Added_Edges(i,1))+1;
                Added_Edges(i,:)=[0 0];
            end
            
            [Added_Edges,Added_Weight,st] = optOddPair(ADJ,DIST,ee,cl,flag,rmflag);
            
%             ee=cl;
%             [Added_Edges,Added_Weight,st] = optOddPair(A,DIST,ee,cl);
%             EdgeTable=table(Added_Edges,diag(DIST(Added_Edges(:,1),Added_Edges(:,2))),'VariableNames',{'EndNodes','Weight'});
%             G_c=addedge(G,EdgeTable);
%             C = conncomp(G_c, 'OutputForm', 'cell');
%             if size(C,2)>1
%                fprintf('Multple Circuits found, could not find a solution') 
%                return;
%             end
%             [~,d]=max(cellfun('length', C));d=find(1:size(C,2)~=d);                    % Temporary fix, selecting the circutit with more nodes
%             Added_Edges(sum(ismember(Added_Edges,C{d}),2)>0,:)=[];
%             A(C{d},:)=0;A(:,C{d})=0;ADJ(C{d},:)=0;ADJ(:,C{d})=0;

%             flag=true;
%             [Added_Edges,Added_Weight,st] = optOddPair(ADJ,DIST,ee,cl,flag);
%             flag=false;
        end
        
        if ~isempty(Added_Edges);add=[add;Added_Edges];end
%         if ~isempty(st)                                             % Newly commented  Remove if needed 
%             if isempty(Start)
%                 Added_Edges(find(st==Added_Edges,1),:)=[];
%             else
%                 Added_Edges(sum(Start==Added_Edges,2)>0,:)=[];  % add longerst when multiple edges are there
%             end
%         end
        for i=1:size(Added_Edges,1)
            if A(Added_Edges(i,1),Added_Edges(i,2))==0
            A(Added_Edges(i,1),Added_Edges(i,2))=DIST(Added_Edges(i,1),Added_Edges(i,2));
            A(Added_Edges(i,2),Added_Edges(i,1))=DIST(Added_Edges(i,1),Added_Edges(i,2));
            end
            ADJ(Added_Edges(i,1),Added_Edges(i,2))=ADJ(Added_Edges(i,1),Added_Edges(i,2))+1;
            ADJ(Added_Edges(i,2),Added_Edges(i,1))=ADJ(Added_Edges(i,2),Added_Edges(i,1))+1;
            Added_Edges(i,:)=[0 0];
        end
        Added_Edges(any(Added_Edges==0,2),:)=[];  
        
        if size(add,1)==1 && isequal(add,st)
            G = graph(A);st=find(1*(mod(degree(G),2)~=0))';
        end
        
%         if isempty(Start)                                         % Newly commented  Remove if needed 
%             if ~isempty(st)
%                 Start=st(1);%cl;
%             else
%                Start=cl; 
%             end
%         end
        if ~isempty(cl)
            Start=cl;
            G = graph(A);
            n=neighbors(G,Start);
            tt=findedge(tempG,Start,n);tt=tt>0;
%             if sum(tt)==0
%                 [~,idx]=max(G.Edges{findedge(G,[Start,Start],[n(1),n(2)]),'Weight'});st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
%                 A(Start,n(idx))=0;A(n(idx),Start)=0;ADJ(Start,n(idx))=ADJ(Start,n(idx))-1;ADJ(n(idx),Start)=ADJ(n(idx),Start)-1;
%             elseif sum(tt)>0
%                 idx=find(G.Edges{findedge(G,Start,n),'Weight'}==max(G.Edges{findedge(G,Start,n(~tt)),'Weight'}));st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
%                 A(st(1),st(2))=0;A(st(2),st(1))=0;ADJ(st(1),st(2))=ADJ(st(1),st(2))-1;ADJ(st(2),st(1))=ADJ(st(2),st(1))-1;
%             end
            if sum(tt)~=numel(n)
                idx=find(G.Edges{findedge(G,Start,n),'Weight'}==max(G.Edges{findedge(G,Start,n(~tt)),'Weight'}));st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
                A(st(1),st(2))=0;A(st(2),st(1))=0;ADJ(st(1),st(2))=ADJ(st(1),st(2))-1;ADJ(st(2),st(1))=ADJ(st(2),st(1))-1;    
            end
        elseif ~isempty(Start)
            Start=Start;
            G = graph(A);
            n=neighbors(G,Start);
            tt=findedge(tempG,Start,n);tt=tt>0;
%             if sum(tt)==0
%                 [~,idx]=max(G.Edges{findedge(G,[Start,Start],[n(1),n(2)]),'Weight'});st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
%                 A(Start,n(idx))=0;A(n(idx),Start)=0;ADJ(Start,n(idx))=ADJ(Start,n(idx))-1;ADJ(n(idx),Start)=ADJ(n(idx),Start)-1;
%             elseif sum(tt)>1
%                 [~,idx]=max(G.Edges{findedge(G,Start,n),'Weight'});st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
%                 A(Start,n(~tt))=0;A(n(~tt),Start)=0;ADJ(Start,n(~tt))=ADJ(Start,n(~tt))-1;ADJ(n(~tt),Start)=ADJ(n(~tt),Start)-1;
%             end
            if sum(tt)~=numel(n)
                idx=find(G.Edges{findedge(G,Start,n),'Weight'}==max(G.Edges{findedge(G,Start,n(~tt)),'Weight'}));st=[G.Edges{findedge(G,Start,n(idx)),'EndNodes'}];
                A(st(1),st(2))=0;A(st(2),st(1))=0;ADJ(st(1),st(2))=ADJ(st(1),st(2))-1;ADJ(st(2),st(1))=ADJ(st(2),st(1))-1; 
            end
        elseif ~isempty(st)
            Start=st(1);
        else
            Start=1;
        end
        
        [Path,Weight] = Fleury(A,ADJ,Start,Added_Edges,Added_Weight);
        if isequal(Path(1:2),st)
            Path=fliplr(Path);
        end
        % [Path,Weight] = Fleury(A,Start);% I added them by myself
end

[V1,V2] = cppTest(A,Path);
%if V1 && V2 == 1                        %Output passes Chinese Postman Test
%     disp(A);                            %Displays matrix
%     Graph(A);                           %Displays graph of matrix
%     fprintf('Optimum path \n');         %Displays Optimum path
%     disp(Path);
%     fprintf('Path weight = %d\n',Weight);    %Displays Optimum weight
%end
end
%--------------------------------------------------------------------%
%Matrix Generator 
%--------------------------------------------------------------------%
function [A,t] = MatGen(Matrix_Input)
switch nargin                           %Switch for input matrix
case 1                                  %Matrix has been inputted
%     disp('Matrix has been entered!')
    A = Matrix_Input;                   %Assign input matrix
    t = EulerTest(A);
otherwise
    t = 0;                              %1 - Eulerian; 2 - Semi; 3 - Random;
    prompt = 'Generate Eulerian Graph? [y/n] ';
    if isempty(prompt)
        prompt = 'y';                   %Defaults to 'y' response
    end
    e = input(prompt,'s');              
    if strcmp(e,'y')== 1                %If Eulerian Graph is requested     
        [n,w,d] = inputs();             %Requests user input
        while t~= 1                     %Loops until Eulerian Graph is generated
            A = EulGraph(n,w,d);        %Generates Eulerian Graph Matrix
            t = EulerTest(A);           %Tests graph type
        end
    else
        prompt = 'Generate Semi-Eulerian Graph? [y/n] ';
        if isempty(prompt)
            prompt = 'y';               %Defaults to 'y' response
        end
        e = input(prompt,'s');
        if strcmp(e,'y')==1             %If Semi-Eulerian Graph is requested 
            [n,w,d] = inputs();         %Requests user input
            while t~=2                  %Loops until graph is Semi-Eulerian
                A = SemiEul(n,w,d);     %Generates Semi-Eulerian Graph Matrix
                t = EulerTest(A);       %Tests graph type
            end
        else                            %If Non-Eulerian Graph is requested 
            [n,w,d] = inputs();         %Requests user input
            while t~=3                  %Loops until graph is Non-Eulerian
                A = Matrix(n,w,d);      %Generates Non-Eulerian Graph Matrix
                t = EulerTest(A);       %Tests graph type
            end
        end
    end
end
end
%--------------------------------------------------------------------%
function [A] = EulGraph(n,w,d)   
while 1==1                              %Loop to ensure generator runs correctly
    A = EulerGen(n,w,d);                %Runs Eulerian Matrix Generator
    if ConnectedTest(A)==1              %Tests if Generated Matrix is Connected
        break                           %Breaks from loop
    end
end
end
%--------------------------------------------------------------------%
function [A] = EulerGen(n,w,d)   
A = zeros(n);                           %Empty matix of size n
if d == 1                               %Undirected Matrix
for r = 1:n                             %Moves down each row sequentially
    for c = 1:n                         %Moves across each column sequentially
        if c == n                       %If calculating the final column
            a = sum(A(r,:));            %Calculates the degree of the current row
            if A(r,r)== 1               %If there is a self loop
                a = a + 1;              %Add an extra dgree to sum
            end
            if mod(a,2) == 0            %If the row degree is even
                A(r,c) = 0;             %Keeps even row degree
            else
                A(r,c) = 1;             %Makes row degree even
            end
        else if c <= r                  %LHS of leading diagonal
                A(r,c) = A(c,r);        %Mirrors the value
            else
                A(r,c) = randi(2)-1;    %Assigns either a 0 or 1
            end
        end
    end
end
else                                    %Generates Directed Eulerian Matrix
while 1==1                              %Continous loop
    A = zeros(n);                       %Empty matix of size n
    deg = randi(n-1,1,n);               %Degrees of each vertex randomly generated
    for iD = 1:n
        tD = deg(1,iD);
        if mod(tD,2)~=0                 %Total degree is odd
            deg(1,iD) = tD+1;           %Make even for Eulerian
        end
    end
        for v = 1:n                     %Current vertex
            inD = sum(lp_solveA(:,v));          %Vertex in-degree
            outD = sum(A(v,:));         %Vertex out-degree
            if (inD+outD)~=2*(deg(1,v)) %Edges haven't been generated
                for C = 1:n             %Generates edges exiting vertex v
                    if A(v,C)==0        %Edge hasn't already been generated
                    if C~=v             %Doesn't generate self-loops
                        vR = n-C+1;     %Remaining vertices to be possibly added 
                        eR = deg(1,v)-sum(A(v,:));   %Remaining edges
                        if vR==eR       %Remaining vertices must have edges
                            A(v,C) = 1;
                        else            %Randomly assign an edge incident to v,C
                            if eR>0     %Edges still remain
                            A(v,C) = randi(2)-1;
                            end
                        end
                    end
                    end
                end
                for R = 1:n             %Generates edges entering vertex v
                    if A(R,v)==0        %Edge hasn't already been generated
                    if R~=v             %Dosn't generate self-loops
                        vR = n-R+1;     %Remaining vertices to be possibly added 
                        eR = deg(1,v)-sum(A(:,v));   %Remaining edges
                        if vR==eR       %Remaining vertices must have edges
                            A(R,v) = 1;
                        else            %Randomly assign an edge incident to C,v
                            if eR>0     %Edges still remain
                            A(R,v) = randi(2)-1;
                            end
                        end
                    end
                    end
                end
            end
        end
        if EulerTest(A)==1              %Eulerian Matrix generated
            break                       %Exit continous loop
        end
end
end
if w ~= 1
    t = randi(w,n);                     %Temp matrix with elements up to max weight
    A = A.*t;                           %Assigns the weights to the eulerian graph
    if d==1                             %Undirected Graph
        A = SymMatrix(A);               %Makes the graph weights symmetrical
    end
end
end
%--------------------------------------------------------------------%
function[A] = SemiEul(n,w,d)
while 1==1                              %Continuous loop 1
    count = 0;                          %Counter
    bV = 0;                             %Break variable
    A = EulerGen(n,w,d);                %Generates an Eulerian Graph
while 1==1                              %Continuous loop 2
    n = size(A,2);                      %No of vertices
    v1 = randi(n);                      %Random row position
    v2 = randi(n);                      %Random column position
    if v1==v2                           %Stops the creation of self loops
        if v1==n
            v2 = v2 - 1;
        else
            v2 = v2 + 1;
        end
    end
    if A(v1,v2) == 0                    %If the element is empty
        A(v1,v2) = 1;                   %Adds new edge, therefore making it Semi-Eulerian
        A(v2,v1) = 1;                   %Undirected graph
        bV = 1;                         %Break Variable changed
        break                           %Exits internal loop
    end
    count = count + 1;                  %Increase Count
    if count == 10
        break
    end
end
if bV==1
    break                               %Exits external loop
end
end
end
%--------------------------------------------------------------------%
function [A] = Matrix(n,w,d)
c = 3;                                  %Density of graph set to 3
A = RandomMatrix(n,c,w);                %Generates requested Matrix
A(logical(eye(size(A)))) = 0;           %Removes Self-Loops
if d == 1
    A = SymMatrix(A);
end
end
%--------------------------------------------------------------------%
function [x] = EulerTest(A)
if A==A'                                %Graph is undirected
    dir = 0;
else                                    %Graph is directed
    dir = 1;
end
n = size(A,1);                        	%Number of vertices
if dir == 0                             %Undirected Graph
    B = BinConv(A);                     %Binary copy of graph A
    deg = [];                           %Vector of odd vertex degrees
    for i = 1:n
        rd = sum(B(i,:));               %Vertex degree
        if mod(rd,2)== 0                %Vertex degree is even
            deg(end+1) = 0;             
        else                            %Vertex degree is odd
            deg(end+1) = 1;
        end
    end
    tD = sum(deg);                      %Number of vertices with odd degrees
    if tD == 0
        x = 1;                          %Graph is Eulerian
    elseif tD == 2 
        x = 2;                          %Graph is Semi-Eulerian
    else
        x = 3;                          %Graph is Non-Eulerian
    end
else
    B = BinConv(A);                     %Binary copy of graph A
    oD = [];                            %Vector of vertex out-degrees
    for i = 1:n
        rd = sum(B(i,:));               %Vertex degree
        oD(end+1) = rd;
    end
    iD = [];                            %Vector of vertex in-degrees
    for i = 1:n
        cd = sum(B(:,i));               %Vertex degree
        iD(end+1) = cd;
    end
    tD = abs(oD - iD);                  %Absolute difference in vertex in and out degrees
    if sum(tD)== 0 
        x = 1;                          %Graph is Eulerian
    elseif sum(tD)== 2
        x = 2;                          %Graph is Semi-Eulerian
    else
        x = 3;                          %Graph is Non-Eulerian        
    end
end    
end
%--------------------------------------------------------------------%
function[A] = RandomMatrix(n,c,w)
if w == 1;
    A = rand(n,n)>(c/10);
else
    B = rand(n,n)>(c/10);
    C = randi([1 w],n,n) ;
    A = B.*C;
end
end
%--------------------------------------------------------------------%
function[A] = SymMatrix(A)
    B = (A + (A'))/2;   %Half the sum of the Matrix and it's Transpose 
    A = round(B);       %Rounds elements of Matrix to Integers
end
%--------------------------------------------------------------------%
function[Cvalue] = ConnectedTest(A)
n=size(A,1); %Calculates the Number of Rows (i.e. Vertices)
B = A^0;
for i = 1:1:n-1; %Loops the equation from i=1 to i-n-1
    B = (B)+(A^(i)); %Sum of A^i
end
if any(B==0); %If any zeros remain in the Matrix sum, Graph is not connected
    %fprintf('Matrix is NOT Connected\n');
    Cvalue = 0;
else %fprintf('Matrix is Connected\n');
    Cvalue = 1;
end
end
%--------------------------------------------------------------------%
function [A] = BinConv(A)
n = size(A,1);              %Size of matrix A
for R = 1:n                 %Runs through all elements in the matrix
    for C = 1:n
        if A(R,C)>0         %If a weighted edge exists
            A(R,C) = 1;     %Replaces it with a 1
        end
    end
end
end
%--------------------------------------------------------------------%
function [n,w,d] = inputs()
    prompt = 'Number of Vertices: ';   	%User inputs
    if isempty(prompt)
        prompt = 10;                 	%Defaults to 10 Vertices
    end
    n = input(prompt);
    prompt = 'Max Weight of Edges: ';
    if isempty(prompt)
        prompt = 1;                   	%Defaults to an unweighted graph
    end
    w = input(prompt);
    prompt = 'Generate Directed Graph [y/n] ';
    if isempty(prompt)
        prompt = 'n';                	%Defaults to an undirected graph
    end
    d = input(prompt,'s');
    d = strcmp(d,'n');                  %0 - Directed  1 - Undirected
end
%--------------------------------------------------------------------%
%--------------------------------------------------------------------%
%Fleury's Algorithm
%--------------------------------------------------------------------%
function [x,w] = Fleury(A,ADJ,r,p,ow)
if A==A'                                %Undirected Graph
    w = sum(sum(A))/2;                  %Total edge weight of undirected graph A
else                                    %Directed Graph
    w = sum(sum(A));                	%Total edge weight of directed graph A
end
A = ADJ;%BinConv(A);                         %Converts the matrix to binary form
x = [];                                 %Empty array to contain list of edges
t = 0;                                  %Test variable to adjust for Shortest Path
switch nargin                           %Switch for number of inputs
    case 5
        R = r;                          %Starts in specified row
        P = p;                          %Edges added according to Shortest Path
        w = w + ow;                     %Adds weight of Shortest Path to total
        t = 1;                          %Changes test variable
    case 3
        R = r;                          %Starts in specified row
    case 2
        R = 1;                          %Starts in row 1 if Eulerain
end
while sum(sum(A))~= 0                   %Runs whilst edges remain
    if t==1
    [A,R,C,P] = FleuryRow(A,R,P);         %Runs Fleury Row function adjusted for Shortest Path   
    else
    [A,R,C] = FleuryRow(A,R);           %Runs Fleury Row function  %,P Vishnu
    end
    x(end+1) = R;                       %Adds the next vertex to the array
    R = C;                              %Sets the next row to be checked to the previous column
end
x(end+1) = C;                           %Adds the final vertex to the array
end
%--------------------------------------------------------------------%
function [A,R,C,lP] = FleuryRow(A,R,P)
if A==A'                                %Undirected Graph
    d = 0;
else                                    %Directed Graph
    d = 1;
end
n = size(A,1);                          %Calculates number of rows
con=[];
switch nargin                           %Switch for number of inputs
    case 3                              %Fleury's adjusted with Shortest Path
        lP = P;                         %Added edges from Shortest Path in List Foramt
    m=max(max(A(R,1:n))); 
    for p = m:-1:1
        con=find(A(R,1:n)==p);               % Vishnu
        if isempty(con)
            continue
        else
            break
        end
    end
    
    if ~isempty(con)
        for C = con%1:n                         %Runs while unchecked columns remain
            if A(R,C)> 0                    %Runs if the edge exists 
                T = 0;                      %Test variable
                ind = 0;                    %Index value
                nP = size(lP,1);            %Edges to be added from Shortest Path
                for h = 1:nP             %Tests every edge in list
                    t1 = lP(h,1);           %Start vertex of current edge
                    t2 = lP(h,2);           %End vertex of current edge
                    if  R==t2 && C==t1 || C==t2 && R==t1   %If edge exists in path
                        T = 1;              %Current edge is present in list
                        ind = h;            %Index of added edge in list
                        lP(h,:) = [0 0];    %Removes edge from list
                        break               %Breaks from for loop
                    end
                end
                Cval = FleuryConn(A,R,C,T);
    %             Cval = isNotBridge2(A,R,C);

                if Cval == 0 && ind ~= 0               %Graph doesn't remain connected
                    lP(ind,:) = [R C];      %Re-adds edge to list
                end
                if Cval == 1 && (sum(A(:,C))>1 || sum(sum(sum(A))==2))  %Runs if A remains connected   %Vishnu modified
                    if T==0       
                    A(R,C)= A(R,C)-1; % 0;              %Removes the edge from A
                    if d == 0               %Undirected Graph
                        A(C,R)= A(C,R)-1; %0;          %Removes back edge from A
                    end
                    end
                    break
                end
            end
        end 
    else
       fprintf('no path found for %d',R) 
    end
    case 2                              %2 Inputs
    for C = 1:n                         %Runs while unchecked columns remain
        if A(R,C)> 0                    %Runs if the edge exists  
            T = 0;
            Cval = FleuryConn(A,R,C,T);
%             Cval = isNotBridge2(A,R,C);
            if Cval == 1                %Graph remains connected ith edge removed
                A(R,C)= A(R,C)-1;% 0;              %Removes the edge from A
                if d == 0               %Undirected Graph       
                    A(C,R)= A(C,R)-1;% 0;          %Removes back edge from A
                end
                break
            end
        end
    end
end
end
%--------------------------------------------------------------------%
function[Cvalue] = FleuryConn(A,R,C,T)
if A==A'                                %Undirected Graph
    d = 0;
else                                    %Directed Graph
    d = 1;
end
if C == R                               %The path is returning to the start vertex
    if sum(sum(A))~= 2                  %The final edge is not the only remaining edge
        if sum(A(C,:))== 1              %There is only one returning edge
            Cvalue = 0;                 %This edge cannot be traversed yet
            return                      %Exit the function
        end
    end
end
if ~T
    A(R,C)=A(R,C)-1;%0;                               %Sets element to zero
    if d == 0
        A(C,R)=A(C,R)-1;%0;                           %For unidirected graphs
    end
end
n=size(A,1);                            %Number of Vertices
B = zeros(n,n);
for i = 1:1:n-1                       %Loops the equation from i=1 to i-n-1
    B = B + (A^(i));                    %Sum of A^i
end
B = IsolatedVertex(B);                  %Removes any isolated vertices
if any(B==0)                          %If any zeros remain in the Matrix sum
    Cvalue = 0;                         %Graph is not connected
else
    Cvalue = 1;                         %Graph is connected
end
end
%--------------------------------------------------------------------%
function [A] = IsolatedVertex(A)
n = size(A,1);                          %Number of Vertices
l = 1:n;                                %List of Vertices
for C = 1:n                             %For each Vertex
    if sum(A(C,:))== 0                  %If the vertex has a degree of zero
        if sum(A(:,C))== 0
            l(l==C) = [];               %Vertex is removed from list
        end
    end
end
A  = A(l,l);                            %Matrix is adjusted to only conatin active vertices
end
%--------------------------------------------------------------------%
%--------------------------------------------------------------------%
%Dijkstra's Algorithm
%--------------------------------------------------------------------%
function [oP,oW] = Dijkstra(A,v1,v2)
Rev = 0;
if v1>v2                               	%If the vertexes are not in numeric order
    temp = v1;  
    v1 = v2;                            %Swap vertices
    v2 = temp;
    Rev = 1;                            %Variable to reverse order before output
end
n = size(A,2);                          %Number of vertices
W = A(v1,:);                            %Start weights to each vertex from vertex 1
for z = 1:n
    if z~=v1                            %Distance from start vertex to itself remains zero
        if W(1,z) == 0                  %If there is no current edge weight
            W(1,z) = inf;               %Infinity is assigned to element
        end
    end
end
P = cell(1,n);                          %Cell array containing optimum paths to each vertex
for j = 1:n
    if A(v1,j)>0                        %Vertices adjacent to start vertex
        P{1,j} = [v1,j];                %Initial paths from start vertex
    end
end
vL = [1:n];                             %Vertor list of each unchecked vertices
for count = 1:n                         %Loops for every vertex
    if count == 1                       %For the first iteration
        R = v1;                         %Start from start vertex
    else                                %Else start from shortest previous edge
        nV(nV==0) = inf;                %Replaces zero elements with inf
        nVt = nV;                       %Temp copy of vertex list
        while 1==1
            [~,ind] = min(nVt);         %Min value and index of next vertex
            if any(vL==ind)==1          %If the vertex hasn't been checked yet
                R = ind;
                break
            else
                nVt(ind)= inf;          %Removes vertex from list
            end
            if any(nVt~=inf)==0         %No adjacent vertices remain to be checked
                mnV = min(vL);          %Next unchecked vertex in numeric order
                R = mnV;
                break
            end
        end
    end
    for C = 1:n                         %Runs through each column
        if A(R,C)== 0
            %Skips calculations if no edge exists
        else
        cPs = P{1,R};                   %Current path to start vertex
        cWs = W(1,R);                   %Current weight to start vertex
        cWe = W(1,C);                   %Current weight to end vertex
        tW = A(R,C);                    %Test weight of edge
        if cWs + tW < cWe               %If a shorter weight exists
            W(1,C) = cWs + tW;          %Replace the weight
            P{1,C} = [cPs,C];           %Replace the path
        end 
        end
    end
    vL(vL==R)=[];                       %Removes checked vertex from list
    nV = A(R,:);                        %List of adjacent vertices
end
oW = W(1,v2);                           %Outputs weight to chosen vertex
oP = cell2mat(P(1,v2));                 %Outputs path to chosen vertex
if Rev== 1                              %Order needs to be reversed
    oP = fliplr(oP);
end
end
%--------------------------------------------------------------------%
%--------------------------------------------------------------------%
%Optimum Pairings od Odd Degree Vertices% modified 
%--------------------------------------------------------------------%
function [x,w,st] = OptPair(A,D)
bA = BinConv(A);                       	%Binary convertion of matrix A
n = size(A,1);                          %Number of vertices
oV = [];                                %List of odd vertices
dir = 0;                                %Variable to identify Digraphs
if bA~=bA'                              %Graph is directed
    dir = 1;
end
for r = 1:n                             %Runs through row 1 to n
    if dir==1
        a = sum(bA(r,:));               %Calculates the out-degree of vertex r
        a = a + sum(bA(:,r));           %Calculates the total degree of vertex r
    else
        a = sum(bA(r,:));               %Calculates the sum of each row
    end
    if bA(r,r)== 1                      %If there are self loops
        a = a + 1;                      %Add an extra degree to vertex
    end
    if mod(a,2) == 1                    %If the row degree is odd
        oV = [oV,r];                    %Adds vertex to list
    end
end
% p = MainPair(oV);                       %All possible combination of pairs
p=get_pairs(oV);                     % I wrote one by myself
w = inf;                                %Weight of optimum pairings
oPp = [];                               %Edges of optimum pairings
m = size(oV,2);                         %Number of odd vertices
l = size(p,1);                          %Number of pairings
for i = 1:l
    count = 0;                          %Sum for each edge pairing
    path = [];                          %Updating optimum path
    weight=[];
    pairs=[];
    for j = 1:2:m-1
        v1 = p(i,j);                    %Vertices in the pair
        v2 = p(i,j+1);
%         [sP,sW] = Dijkstra(A,v1,v2); 
                                        % Add the straight line insdead
        sW=D(v1,v2);
        weight=[weight,sW];
        sP=[v1 v2];
        count = count + sW;             %Add shortest weight to sum
        path = [path,sP,0];             %Adds edges to path
        pairs=[pairs;sP];
    end
    %check connectivity by myself...
    pairs=[pairs;pairs(:,2),pairs(:,1)];
    B = full(sparse(pairs(:, 1), pairs(:, 2), 1, n, n));
    B = B+A>0;
   
B(1:n+1:end) = 1;
    [~,~,r,~] = dmperm(B);
    if length(r)>2
        continue;
    end
    [~,Inx]=max(weight);
    weight(Inx)=[];
    weight=sum(weight);
    temp=path((Inx-1)*3+1:(Inx-1)*3+2);%start and target nodes
%     path((Inx-1)*3+1:(Inx-1)*3+2)=0;
    if weight<w%count<w                          %If the new weight is less than
        w = weight;%count;                      %Update weight value
        oPp = path;                     %Update path
        st=temp;%start and target nodes
    end
    
    
end
n = size(oPp,2);                        %Vertices in path (with zeros)
x = [];                                 %Matrix list of edges in additional path
for i = 1:n-1
    a = oPp(1,i);                       %Consecutive vertices in path (Edge)
    b = oPp(1,i+1);
    x = [x;a,b];                        %Adds edge to list
end
x(any(x==0,2),:)=[];                    %Removes the zero break edges
end
%--------------------------------------------------------------------%
function [x] = MainPair(oV)
n = size(oV,2);                         %Number of odd vertices
if n == 2                               %2 odd vertices
    x = oV;
end
if n == 4                               %4 odd vertices
    x = [];
    for i = 2:n
        [x1,oVtn] = GenPair(oV,i);
        x = [x;x1,oVtn];
    end
end
if n == 6                               %6 odd vertices
    x = [];
    for i = 2:n
        [x1,oVtn] = GenPair(oV,i);
        m = size(oVtn,2);
        for j = 2:m
            [x2,oVtn2] = GenPair(oVtn,j,x1);
            x = [x;x2,oVtn2];
        end
    end
end
if n == 8                               %8 odd vertices
    x = [];
    for i = 2:n
        [x1,oVtn] = GenPair(oV,i);
        m = size(oVtn,2);
        for j = 2:m
            [x2,oVtn2] = GenPair(oVtn,j,x1);
            o = size(oVtn2,2);
            for k = 2:o
                [x3,oVtn3] = GenPair(oVtn2,k,x2);
                x = [x;x3,oVtn3];
            end
        end
    end
end
if n == 10                               %10 odd vertices
    x = [];
    for i = 2:n
        [x1,oVtn] = GenPair(oV,i);
        m = size(oVtn,2);
        for j = 2:m
            [x2,oVtn2] = GenPair(oVtn,j,x1);
            o = size(oVtn2,2);
            for k = 2:o
                [x3,oVtn3] = GenPair(oVtn2,k,x2);
                p = size(oVtn3,2);
                for q = 2:p
                [x4,oVtn4] = GenPair(oVtn3,q,x3);
                x = [x;x4,oVtn4];
                end
            end
        end
    end
end
end
%--------------------------------------------------------------------%
function [x,oVtn] = GenPair(oVtp,i,x1)
oVtn = oVtp;                            %Temp copy of remaining odd vertices
a = oVtn(1,1);                          %Value of first vertex remaining in list
oVtn(:,i)=[];                           %Deletes current vertex pairing
oVtn(:,1)=[];
switch nargin                           %Switch for number of inputs
    case 3                              %If a previous group of pairings is entered
        x = [x1,a,i];                   %Start pairings for next loop, with previous pairings
    case 2                              %For first iteration
        x = [a,i];                      %Start pairings for next loop
end
end
%--------------------------------------------------------------------%
function [Added_Edges,Added_Weight,st] = optOddPair(A,DIST,ee,cl,flag,rmflag) % Vishnu: Pairing the odd nodes using LinearProgramming

G = graph(A);
st=[];
%Iteration1 
b_n=1*(mod(sum(A,2),2)~=0);

if any(b_n)
    Iter1 = linearProg(G,b_n,DIST,ee,flag,rmflag);
else
    Iter1=[];
end

if ~ismatrix(ee)
    if any(ismember(ee,Iter1(:)))
        st= Iter1(ee(ismember(ee,Iter1(:)))==Iter1(:,1),:);
        if isempty(st)
            st= Iter1(ee(ismember(ee,Iter1(:)))==Iter1(:,2),:);
        end
    elseif ~isempty(cl)
        if sum(sum(Iter1==cl))>=1
            ii=Iter1==cl;
            st=Iter1(ii(:,1),:);
            if isempty(st)
                st= Iter1(ii(:,2),:);
            end
            st=st(1,:);
        end
    else
        [~,s]=max(diag(DIST(Iter1(:,1),Iter1(:,2))));
        st=Iter1(s,:);      % Start Edge
    end
else
    if ~isempty(Iter1)
        [~,s]=max(diag(DIST(Iter1(:,1),Iter1(:,2))));
        st=Iter1(s,:);      % Start Edge
    else
        st=[];
    end
end

%Iteration2
% b_n(st(:))=0;
% Added_Edges = linearProg(G,b_n,DIST);

% Added_Edges=Iter1(1:end~=s,:); %Remove st from the Added_Edges 
Added_Edges=Iter1;

if ~isempty(Added_Edges)
    Added_Weight =sum(diag(DIST(Added_Edges(:,1),Added_Edges(:,2))));
else
    Added_Weight=[];
end

end
%--------------------------------------------------------------------%
function [Added_Edges] = linearProg(G,b_n,DIST,ee,flag,rmflag) % Vishnu: LinearProgramming

%%% Combining the all egdes and Generatign the Path, LP Problem
% bA = BinConv(A); 
Added_Edges=[];
combEdge = combnk(find(b_n),2);                             % Create all combinations of edges b/w odd nodes

if rmflag
combEdge(ismember(combEdge,G.Edges.EndNodes,'rows'),:)=[];  % Removing edges from crackEdge
end

%%%Test % to solve 3edge one intersection problem
if ~isequal(unique(find(b_n)),unique(combEdge(:)))
    combEdge = combnk(find(b_n),2);                             % Create all combinations of edges b/w odd nodes
end
%%%

if size(ee,2)>1 && ~isempty(ee)
    combEdge(ismember(combEdge,ee,'rows'),:)=[];
end
c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));

if flag
    Gp=graph(combEdge(:,1),combEdge(:,2),c_e);
    T = minspantree(Gp);
    T.Edges

    G1=addedge(G,T.Edges);G1=simplify(G1);
    Added_Edges=[Added_Edges;G.Edges.EndNodes];
    b_n=1*(mod(degree(G1),2)~=0);
    combEdge = combnk(find(b_n),2);
    c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));
end


% if ~isempty(ee)
%     combEdge(sum(ismember(combEdge,ee),2)>0,:)=[];
% end

No_node=height(G.Nodes);
No_pairedge=size(combEdge,1);

% G_p = graph(combEdge(:, 1), combEdge(:, 2));

a_ne = zeros(No_node, No_pairedge);
for i=1:size(combEdge,1)
    a_ne(combEdge(i,[1,2]),i)=1;
end

% % b_n(setdiff(1:end,unique(combEdge(:))))=0;
% if mod(length(ee),2)==0                                     % Removing continuing node while keeping even number of odd nodes.
%     b_n(ee)=0;
% elseif mod(length(ee),2)~=0 && length(ee)~=1
%     b_n(ee(1:end-1))=0;
% end
%w_n=-2*ones(No_node);

% x_e = intlinprog ([c_e;zeros(No_node,1)],No_pairedge,[],[],[a_ne,w_n], b_n,zeros(1,No_pairedge),inf);
options = optimoptions('intlinprog','Display','off');
x_e = intlinprog (c_e,1:No_pairedge,[],[],a_ne, b_n,zeros(1,No_pairedge),ones(1,No_pairedge),options);%

% options = optimoptions('linprog','Algorithm','dual-simplex')

if ~isempty(x_e)
Added_Edges=[Added_Edges;combEdge(logical(x_e(1:No_pairedge)),:)];
end

end

function [Added_Edges] = linearProgRRP(G,b_n,combEdge,DIST,ee) % Vishnu: LinearProgramming

%%% Combining the all egdes and Generatign the Path, LP Problem
% bA = BinConv(A); 
Added_Edges=[];
combEdge(ismember(combEdge,G.Edges.EndNodes,'rows'),:)=[];  % Removing edges from crackEdge
if size(ee,2)>1 && ~isempty(ee)
    combEdge(ismember(combEdge,ee,'rows'),:)=[];
end
c_e=diag(DIST(combEdge(:,1),combEdge(:,2)));

No_node=height(G.Nodes);
No_pairedge=size(combEdge,1);

a_ne = zeros(No_node, No_pairedge);
for i=1:size(combEdge,1)
    a_ne(combEdge(i,[1,2]),i)=1;
end

% x_e = intlinprog ([c_e;zeros(No_node,1)],No_pairedge,[],[],[a_ne,w_n], b_n,zeros(1,No_pairedge),inf);
x_e = intlinprog (c_e,1:No_pairedge,[],[],a_ne, b_n,zeros(1,No_pairedge),ones(1,No_pairedge));

% options = optimoptions('linprog','Algorithm','dual-simplex')


Added_Edges=[Added_Edges;combEdge(logical(x_e(1:No_pairedge)),:)];

end

%--------------------------------------------------------------------%
%--------------------------------------------------------------------%
%Chinese Postman Problem Solution Test
%--------------------------------------------------------------------%
function [V1,V2] = cppTest(A,P)
if A==A'                        %If graph is undirected
    Dir = 0;                    %Graph is undirected
else
    Dir = 1;                    %Graph is directed
end
aL = [];                        %List of edges in matrix
pL = [];                        %List of edges in path
aN = size(A,1);                 %Number of vertices in graph
pN = size(P,2);                 %Number of vertices in path
for R = 1:aN
    for C = 1:aN
        if A(R,C) > 0           %If edge exists
            aL = [aL;R,C];      %Adds to matrix list
            if Dir == 0         %If the graph is undirected
                A(C,R) = 0;     %Removes back edge
            end
        end
    end
end
for i = 1:pN-1
    aP = P(1,i);                %Consecutive vertices (each edge)
    bP = P(1,i+1);
    pL = [pL;aP,bP];            %Adds edge to matrix list
end
if Dir==0                       %If graph is undirected
    fpL = fliplr(pL);           %Return edges of matrix
    pL = [pL;fpL];
end
L = setdiff(aL,pL,'rows');      %Edges not in path
v = size(L,1);                  %Number of edges not traversed
if v==0                         
    %disp('All edges traversed');
    V1 = 1;
else
    disp('Edges NOT traversed');
    disp(L);
    V1 = 0;
end
K = setdiff(pL,aL,'rows');
u = size(K,1);                  %Number of edges traversed that don't exists
if u==0                         
    disp('Edges traversed that dont exists!');
    disp(K);
    V2 = 0;
else
    %disp('All edges exists!');
    V2 = 1;
end
end
%--------------------------------------------------------------------%
%--------------------------------------------------------------------%
%Graphing Function
%--------------------------------------------------------------------%
function [] = Graph(A)
n = size(A,1);
xy = [];
for v = 0:n
    x = 1+v*((n+1)/n);
    if mod(v,2)==0
        y = (n*(n + 1) - sqrt((2*(n^3)*(2*v + 1)) - (n^2)*(4*(v^2) + 1) - 4*n*v*(2*v + 1) - 4*(v^2)))/(2*n);
    else
        y = (sqrt((2*(n^3)*(2*v + 1)) - (n^2)*(4*(v^2) + 1) - 4*n*v*(2*v + 1) - 4*(v^2)) + n*(n + 1))/(2*n);
    end
    xy = [xy;x y];
end

gplot(A,xy,'-ok');
axis([0 n+1 0 n+1]);
end
%--------------------------------------------------------------------%
%--------------------------------------------------------------------%