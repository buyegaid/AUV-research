function actor = createActor(obsInfo, actInfo)

statePath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization','none','Name','state')
    fullyConnectedLayer(128,'Name','fc1')
    reluLayer
    fullyConnectedLayer(128,'Name','fc2')
    reluLayer
    fullyConnectedLayer(actInfo.Dimension(1),'Name','fc3')
    tanhLayer
    ];

actorNet = layerGraph(statePath);

actorOpts = rlRepresentationOptions( ...
    'LearnRate',1e-4, ...
    'GradientThreshold',1);

actor = rlDeterministicActorRepresentation( ...
    actorNet, obsInfo, actInfo, ...
    'Observation','state', actorOpts);

end