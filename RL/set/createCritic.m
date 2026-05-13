function critic = createCritic(obsInfo, actInfo)

statePath = [
    featureInputLayer(obsInfo.Dimension(1),'Name','state')
    fullyConnectedLayer(128,'Name','fc1')
    reluLayer
    fullyConnectedLayer(128,'Name','fc2')
    ];

actionPath = [
    featureInputLayer(actInfo.Dimension(1),'Name','action')
    fullyConnectedLayer(128,'Name','fc3')
    ];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer
    fullyConnectedLayer(1,'Name','Qvalue')
    ];

criticNet = layerGraph(statePath);
criticNet = addLayers(criticNet, actionPath);
criticNet = addLayers(criticNet, commonPath);

criticNet = connectLayers(criticNet,'fc2','add/in1');
criticNet = connectLayers(criticNet,'fc3','add/in2');

criticOpts = rlRepresentationOptions( ...
    'LearnRate',1e-3, ...
    'GradientThreshold',1);

critic = rlQValueRepresentation( ...
    criticNet, obsInfo, actInfo, ...
    'Observation','state','Action','action',criticOpts);

end