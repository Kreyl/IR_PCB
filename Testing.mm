<map version="freeplane 1.11.5">
<!--To view this file, download free mind mapping software Freeplane from https://www.freeplane.org -->
<node TEXT="Тестирование" FOLDED="false" ID="ID_1723255651" CREATED="1283093380553" MODIFIED="1700164938851"><hook NAME="MapStyle">
    <properties fit_to_viewport="false;" associatedTemplateLocation="template:/KL_standard-1.6.mm"/>

<map_styles>
<stylenode LOCALIZED_TEXT="styles.root_node" STYLE="oval" UNIFORM_SHAPE="true" VGAP_QUANTITY="24 pt">
<font SIZE="24"/>
<stylenode LOCALIZED_TEXT="styles.predefined" POSITION="bottom_or_right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="default" ID="ID_1008054917" MAX_WIDTH="600 px" COLOR="#000000" STYLE="bubble">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" DASH="" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1008054917" STARTARROW="NONE" ENDARROW="DEFAULT"/>
<font NAME="SansSerif" SIZE="10" BOLD="false" ITALIC="false"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.details"/>
<stylenode LOCALIZED_TEXT="defaultstyle.attributes">
<font SIZE="9"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.note"/>
<stylenode LOCALIZED_TEXT="defaultstyle.floating">
<edge STYLE="hide_edge"/>
<cloud COLOR="#f0f0f0" SHAPE="ROUND_RECT"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.selection" BACKGROUND_COLOR="#4e85f8" BORDER_COLOR_LIKE_EDGE="false" BORDER_COLOR="#4e85f8"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.user-defined" POSITION="bottom_or_right" STYLE="bubble">
<stylenode TEXT="OK">
<icon BUILTIN="button_ok"/>
</stylenode>
<stylenode TEXT="NotOk">
<icon BUILTIN="button_cancel"/>
</stylenode>
<stylenode TEXT="Question">
<icon BUILTIN="help"/>
</stylenode>
<stylenode TEXT="Attention">
<icon BUILTIN="yes"/>
</stylenode>
<stylenode TEXT="Stopper">
<icon BUILTIN="stop-sign"/>
</stylenode>
<stylenode TEXT="Waiting">
<icon BUILTIN="hourglass"/>
</stylenode>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.AutomaticLayout" POSITION="bottom_or_right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="AutomaticLayout.level.root" COLOR="#000000">
<font SIZE="18"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,1" BACKGROUND_COLOR="#ccffcc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,2" BACKGROUND_COLOR="#ffffcc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,3" BACKGROUND_COLOR="#ffcccc"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,4" COLOR="#111111">
<font SIZE="10"/>
</stylenode>
</stylenode>
</stylenode>
</map_styles>
</hook>
<node TEXT="Что тестить" POSITION="bottom_or_right" ID="ID_1026893008" CREATED="1700164941297" MODIFIED="1700164944640">
<node TEXT="Кварц" ID="ID_993072251" CREATED="1700165518024" MODIFIED="1700165523200">
<node TEXT="Если плохо - то мигать Side LEDs" ID="ID_864749169" CREATED="1700165523889" MODIFIED="1700165546597"/>
</node>
<node TEXT="Flash" ID="ID_1085714971" CREATED="1700165104529" MODIFIED="1700165107860">
<node TEXT="Если плохо - то мигать Side LEDs" ID="ID_319256265" CREATED="1700165523889" MODIFIED="1700165546597"/>
</node>
<node TEXT="GPIOs" ID="ID_1599920865" CREATED="1700164945351" MODIFIED="1700164990460">
<node TEXT="Подкл синие диоды, зажигать поочерёдно" ID="ID_1879649791" CREATED="1700165161100" MODIFIED="1700165191381"/>
</node>
<node TEXT="NeoPix" ID="ID_385653959" CREATED="1700165034685" MODIFIED="1700165039228">
<node TEXT="Подкл ленту, зажигать поочерёдно" ID="ID_1264727634" CREATED="1700165186546" MODIFIED="1700165222191"/>
</node>
<node TEXT="IR TX" ID="ID_1949179470" CREATED="1700165044315" MODIFIED="1700165050890">
<node TEXT="Передавать что-то" ID="ID_1003326266" CREATED="1700165228231" MODIFIED="1700165236196"/>
</node>
<node TEXT="IR RCVR" ID="ID_1677853643" CREATED="1700165040837" MODIFIED="1700165043976">
<node TEXT="Принимать переданное" ID="ID_1234318159" CREATED="1700165614569" MODIFIED="1700165619973"/>
</node>
<node TEXT="LEDs" ID="ID_583402215" CREATED="1700165058273" MODIFIED="1700165065973">
<node TEXT="Side" ID="ID_1368803402" CREATED="1700165077169" MODIFIED="1700165080950">
<node TEXT="Поочерёдно" ID="ID_662225449" CREATED="1700165632196" MODIFIED="1700165637492"/>
</node>
<node TEXT="Front" ID="ID_1771549333" CREATED="1700165081168" MODIFIED="1700165083484">
<node TEXT="Поочерёдно" ID="ID_1796295379" CREATED="1700165638983" MODIFIED="1700165642720"/>
</node>
<node TEXT="System" ID="ID_1406848643" CREATED="1700165084123" MODIFIED="1700165088383">
<node TEXT="Мигать" ID="ID_337114083" CREATED="1700165628548" MODIFIED="1700165630469"/>
</node>
</node>
<node TEXT="Audio" ID="ID_211743750" CREATED="1700165108159" MODIFIED="1700165126861">
<node TEXT="Играть синус" ID="ID_426115060" CREATED="1700165646112" MODIFIED="1700165652045"/>
</node>
<node TEXT="Buzzer" ID="ID_113275115" CREATED="1700165134437" MODIFIED="1700165137657">
<node TEXT="Играть выстрел" ID="ID_1485199114" CREATED="1700165653638" MODIFIED="1700165659781"/>
</node>
<node TEXT="USB" ID="ID_1031199562" CREATED="1700165006637" MODIFIED="1700165008879">
<node TEXT="Придётся воткнуть кабель" ID="ID_565232366" CREATED="1700165557385" MODIFIED="1700165585999"/>
</node>
</node>
</node>
</map>
