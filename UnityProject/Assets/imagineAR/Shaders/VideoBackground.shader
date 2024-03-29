﻿Shader "imagineAR/VideoBackground" {
    Properties {
        _MainTex ("Base (RGB)", 2D) = "white" {}
    }
    SubShader {
        Tags {"Queue"="geometry-11" "RenderType"="opaque" }
        Pass {
            ZWrite Off
            Cull Off
            Lighting Off

            SetTexture [_MainTex] {
                combine texture
            }
        }
    }
}